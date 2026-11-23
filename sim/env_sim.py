"""Environment simulator, v2: solar from ERA5, load from Starlink + speed, battery in Wh -> V.

Pretends to be Scout's power-monitoring board, a separate MAVLink component
(sysid 1, compid 180 = MAV_COMP_ID_BATTERY) plugged into the autopilot's SERIAL1.
ArduPilot forwards its messages to its other ports, so the service and QGC see them.

  reads from SITL : SYSTEM_TIME (scenario clock), SERVO_OUTPUT_RAW (throttle, Starlink on output 9),
                    VFR_HUD (speed), HEARTBEAT (armed, mode)
  reads from disk : data/weather/truth.csv (ERA5, hourly)  <- the env sim is the ONLY reader of the truth
  sends           : HEARTBEAT (type BATTERY) every 1 s wall time
                    BATTERY_STATUS id 0/1/2 every 1 s of autopilot time
                      0 = battery (net, signed)   1 = total load   2 = solar
  logs            : sim/logs/env_sim.csv, one row per minute of sim time (the "truth" for later comparison)
  state           : sim/state/world.json, rewritten atomically every 10 s of sim time (t, battery, lat/lon, Starlink).
                    Loaded on start if it exists, so a killed SITL + env sim can carry on from there
                    (sim/run_sitl.sh --resume starts SITL at the saved time and place).

Survives SITL restarts: if the autopilot goes quiet for LINK_TIMEOUT_S, the env sim reconnects to
tcp:5762 and waits for it. Time that doesn't move forward (no SYSTEM_TIME yet, 0, or backwards) is
not integrated, and neither is a jump of more than MAX_STEP_S.

This is the TRUTH model, with the notebooks' latest energy model (nb 13): load = 10 W + 22 W Starlink
+ k_mode v^2 (nb 08), battery nb 11 B1, solar 0.164 m^2 x irradiance (nb 10).
Forecasts still can't be trivially perfect: the truth uses ERA5 solar and the real throttle/speed,
the planner uses its own solar curve and the mission plan.

Run from the repo root (SITL running via sim/run_sitl.sh):
    python3 sim/env_sim.py            # resume from sim/state/world.json if it exists
    python3 sim/env_sim.py --fresh    # ignore the saved state, start at V_START
"""
import argparse
import csv
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd
from pymavlink import mavutil

# ---------------- model parameters (from the notebooks; see claude/NOTES.md) ----------------
SOLAR_K_M2 = 0.164          # P_solar = k * G, effective area eta*A (nb 10 Part 4)
HOTEL_W = 10.0              # base load (nb 06)
STARLINK_W = 22.0           # extra while Starlink is on (nb 06)
# propulsion = k_mode * v**2 (nb 08 section 2 / nb 13 4.3), W per (m/s)^2, v = speed over ground.
# Keyed by ArduPilot Rover custom_mode. Mode 17 never shows up in SITL but is kept to match the model.
ROVER_MODE_MANUAL, ROVER_MODE_HOLD, ROVER_MODE_AUTO = 0, 4, 10
K_PROP_MODE = {ROVER_MODE_AUTO: 62.7, 17: 60.6}
K_PROP_OTHER = K_PROP_MODE[ROVER_MODE_AUTO]   # GUIDED, RTL, LOITER...: the model has no k, the motor drives, use AUTO's
# Motor counts as driving when armed and not in MANUAL or HOLD (nb 13: MANUAL = motor off).
# Not a throttle threshold: SITL's motorboat holds ~0.6 m/s at <0.5 % throttle, so a throttle
# gate dropped the propulsion load entirely.
# battery: nb 11 Model B1 (best free run with measured inputs; used by nb 12), copied 20 Nov
BATT_K_WH_PER_V = 420.3     # Wh per volt of open-circuit voltage
BATT_ETA = 1.138            # solar -> battery factor (fitted; >1 is unphysical, see nb 11 notes)
BATT_P_DRAIN_W = -0.312     # constant drain term (fitted)
R_INTERNAL_OHM = 0.03921
V_CAP = 25.0                # charge cap: no charging above 25 V terminal, extra solar is lost
V_EMPTY = 20.0              # assumed cut-off (~2.86 V/cell for 7S); below it the boat is flat
V_START = 23.53             # telemetry median, first 2 h of 14 Jul
# ---------------------------------------------------------------------------------------------

AUTOPILOT_COMP = 1
OWN_COMP = mavutil.mavlink.MAV_COMP_ID_BATTERY   # 180
PUBLISH_EVERY_S = 1.0                            # autopilot time
LOG_EVERY_S = 60.0                               # autopilot time
SAVE_EVERY_S = 10.0                              # autopilot time; run_sitl.sh --resume adds RESUME_GAP_S on top
MAX_STEP_S = 120.0                               # bigger clock jumps (SITL restart) are not integrated
LINK_TIMEOUT_S = 5.0                             # wall time without autopilot messages -> reconnect
UNUSED_CELL = 65535


class WeatherTruth:
    """ERA5 at one point, hourly. Open-Meteo irradiance is the mean over the hour BEFORE each
    timestamp, so each value is placed at the middle of its hour and interpolated linearly."""

    def __init__(self, path):
        df = pd.read_csv(path, comment="#", parse_dates=["time"])
        mid = df["time"] - pd.Timedelta(minutes=30)
        self.t = mid.map(lambda x: x.timestamp()).to_numpy()
        self.g = df["shortwave_radiation"].to_numpy(dtype=float)
        print(f"env_sim: weather {df['time'].iloc[0]} -> {df['time'].iloc[-1]} ({len(df)} hours)")

    def irradiance(self, t_unix):
        return float(np.interp(t_unix, self.t, self.g, left=0.0, right=0.0))


class Battery:
    """nb 11 free_run(), step by step: OCV integrates net power / k, terminal V = OCV + R*I,
    charge cap at V_CAP (surplus solar lost)."""

    def __init__(self, v_start):
        self.ocv = v_start            # first step treats the start voltage as OCV (I unknown)
        self.v = v_start
        self.e_wh = 0.0               # energy change since start (for the log)

    @classmethod
    def from_state(cls, st):
        b = cls(st["batt_v"])
        b.ocv, b.e_wh = st["ocv_v"], st["energy_wh"]
        return b

    def step(self, p_solar_w, p_load_w, dt_s):
        """Returns (p_net accepted by the battery, terminal V, battery current)."""
        p_net = BATT_ETA * p_solar_w - p_load_w - BATT_P_DRAIN_W
        i_b = p_net / self.v                                # current from last terminal V
        v = self.ocv + R_INTERNAL_OHM * i_b
        if p_net > 0 and v > V_CAP:                         # full: limit charge current
            i_b = max((V_CAP - self.ocv) / R_INTERNAL_OHM, 0.0)
            v = max(self.ocv + R_INTERNAL_OHM * i_b, self.ocv)
            p_net = i_b * v
        if p_net < 0 and self.ocv <= V_EMPTY:               # flat: nothing left to draw
            p_net, i_b, v = 0.0, 0.0, self.ocv
        self.ocv += p_net * dt_s / 3600 / BATT_K_WH_PER_V
        self.e_wh += p_net * dt_s / 3600
        self.v = v
        return p_net, v, i_b


class EnvSim:
    def __init__(self, conn, weather, log_path, state_path, fresh=False):
        self.conn = conn
        self.m = None
        self.weather = weather
        self.state_path = state_path
        self.batt = Battery(V_START)
        self.t_unix = 0.0
        self.lat = self.lon = None
        self.throttle_pwm = 0
        self.starlink_pwm = 0
        self.speed = 0.0
        self.armed = False
        self.mode = None
        self.last_step_t = None
        self.last_rx_wall = 0.0
        self.out = {}
        if state_path.exists() and not fresh:
            st = json.loads(state_path.read_text())
            self.batt = Battery.from_state(st)
            self.last_step_t = st["t_unix"]          # SITL must restart at (or just after) this time
            self.lat, self.lon = st.get("lat"), st.get("lon")
            print(f"env_sim: resumed from {state_path}: {st['time_utc']}  "
                  f"OCV {self.batt.ocv:.3f} V  ({self.lat}, {self.lon})")
        else:
            print(f"env_sim: fresh start, V = {V_START} V")
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_f = open(log_path, "a", newline="")
        self.log = csv.writer(self.log_f)
        if log_path.stat().st_size == 0:
            self.log.writerow(["t_unix", "time_utc", "irradiance_wm2", "solar_w", "load_w", "prop_w",
                               "starlink", "speed_ms", "throttle_pct", "batt_w", "energy_wh", "ocv_v", "batt_v"])

    # ---------- MAVLink ----------
    def connect(self):
        """(Re)open the link and wait for the autopilot. Loops until SITL is up, so it also
        covers SITL being killed and restarted."""
        if self.m is not None:
            try:
                self.m.close()
            except OSError:
                pass
            self.m = None
        print(f"env_sim: connecting to {self.conn} ...")
        while self.m is None:
            try:
                self.m = mavutil.mavlink_connection(self.conn, source_system=1,
                                                    source_component=OWN_COMP, retries=0)
            except OSError:
                time.sleep(1.0)                   # SITL not listening yet
        print("env_sim: waiting for autopilot heartbeat...")
        while True:   # wait for the *autopilot's* heartbeat (ignore GCS, etc.)
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if hb is None:
                self.heartbeat()                  # let SITL route to us
                continue
            if hb.get_srcComponent() == AUTOPILOT_COMP and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), AUTOPILOT_COMP
                break
        print(f"env_sim: connected to system {self.m.target_system}")
        self.last_rx_wall = time.monotonic()
        # stream rates are lost when SITL restarts, so ask again on every connect
        for name in ("SYSTEM_TIME", "SERVO_OUTPUT_RAW", "VFR_HUD", "GLOBAL_POSITION_INT"):
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")
            self.m.mav.command_long_send(self.m.target_system, AUTOPILOT_COMP,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         msg_id, 1e6, 0, 0, 0, 0, 0)   # 1 Hz

    def read(self):
        """Read one message. Returns False if the link is dead (caller reconnects)."""
        try:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
        except (OSError, ConnectionError):
            return False
        if msg is None or msg.get_srcComponent() != AUTOPILOT_COMP:
            return time.monotonic() - self.last_rx_wall < LINK_TIMEOUT_S
        self.last_rx_wall = time.monotonic()
        t = msg.get_type()
        if t == "SYSTEM_TIME" and msg.time_unix_usec:
            self.t_unix = msg.time_unix_usec / 1e6
        elif t == "GLOBAL_POSITION_INT" and (msg.lat or msg.lon):
            self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
        elif t == "SERVO_OUTPUT_RAW":
            self.throttle_pwm = msg.servo3_raw
            self.starlink_pwm = getattr(msg, "servo9_raw", 0)
        elif t == "VFR_HUD":
            self.speed = msg.groundspeed
        elif t == "HEARTBEAT" and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode = msg.custom_mode
        return True

    def send(self, fn):
        """Send, ignoring a dead socket (read() notices and reconnects)."""
        try:
            fn()
        except (OSError, ConnectionError):
            pass

    def heartbeat(self):
        self.send(lambda: self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_BATTERY,
                                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                    mavutil.mavlink.MAV_STATE_ACTIVE))

    def send_battery(self, batt_id, volts, amps, remaining=-1):
        cells = [round(volts * 1000)] + [UNUSED_CELL] * 9            # mV, slot 0 only
        self.send(lambda: self.m.mav.battery_status_send(
            batt_id, mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL, mavutil.mavlink.MAV_BATTERY_TYPE_UNKNOWN,
            32767, cells, round(amps * 100), -1, -1, remaining))

    # ---------- physics ----------
    @property
    def throttle_pct(self):
        return (self.throttle_pwm - 1500) / 5 if self.throttle_pwm else 0.0

    @property
    def starlink_on(self):
        # 0 = output never commanded yet; SERVO9_TRIM 1900 means the default is ON
        return self.starlink_pwm == 0 or self.starlink_pwm > 1500

    @property
    def motor_driving(self):
        return self.armed and self.mode not in (ROVER_MODE_MANUAL, ROVER_MODE_HOLD)

    @property
    def k_prop(self):
        return K_PROP_MODE.get(self.mode, K_PROP_OTHER)

    def step(self):
        """Advance the physics to the current autopilot time."""
        dt = 0.0 if self.last_step_t is None else self.t_unix - self.last_step_t
        if dt < 0 or dt > MAX_STEP_S:
            print(f"env_sim: clock jumped {dt:+.0f} s, not integrated")
            dt = 0.0                                      # restart / fresh SITL: don't integrate
        self.last_step_t = self.t_unix

        g = self.weather.irradiance(self.t_unix)
        p_solar = SOLAR_K_M2 * g
        p_prop = self.k_prop * self.speed ** 2 if self.motor_driving else 0.0
        p_load = HOTEL_W + STARLINK_W * self.starlink_on + p_prop
        p_uncapped = BATT_ETA * p_solar - p_load - BATT_P_DRAIN_W
        p_net, v_bus, i_batt = self.batt.step(p_solar, p_load, dt)
        if p_net < p_uncapped - 1e-9:                       # battery full: panels curtailed
            p_solar = max(0.0, (p_net + p_load + BATT_P_DRAIN_W) / BATT_ETA)
        self.out = dict(g=g, p_solar=p_solar, p_load=p_load, p_prop=p_prop, p_batt=p_net,
                        v=v_bus, i_batt=i_batt, i_load=p_load / v_bus, i_solar=p_solar / v_bus)

    def publish(self):
        o = self.out
        pct = int(max(0, min(100, 20 * o["v"] - 400)))     # same formula as the real telemetry
        self.send_battery(0, o["v"], o["i_batt"], remaining=pct)
        self.send_battery(1, o["v"], o["i_load"])
        self.send_battery(2, o["v"], o["i_solar"])

    def save_state(self):
        """Atomic: write a temp file, then rename over world.json (a kill never leaves half a file)."""
        st = dict(t_unix=round(self.t_unix, 3),
                  time_utc=datetime.fromtimestamp(self.t_unix, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                  ocv_v=self.batt.ocv, batt_v=self.batt.v, energy_wh=self.batt.e_wh,
                  lat=self.lat, lon=self.lon, starlink_on=bool(self.starlink_on))
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_path.with_suffix(".json.tmp")
        with open(tmp, "w") as f:
            json.dump(st, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.state_path)

    def write_log(self):
        o = self.out
        ts = datetime.fromtimestamp(self.t_unix, timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
        self.log.writerow([round(self.t_unix), ts, round(o["g"], 1), round(o["p_solar"], 1),
                           round(o["p_load"], 1), round(o["p_prop"], 1), int(self.starlink_on),
                           round(self.speed, 2), round(self.throttle_pct, 1), round(o["p_batt"], 1),
                           round(self.batt.e_wh, 2), round(self.batt.ocv, 3), round(o["v"], 3)])
        self.log_f.flush()
        print(f"{ts}  G={o['g']:5.0f} W/m2  solar={o['p_solar']:5.1f} W  load={o['p_load']:5.1f} W "
              f"(prop {o['p_prop']:4.1f}, starlink {'on' if self.starlink_on else 'off'})  "
              f"batt={o['p_batt']:+6.1f} W  {o['v']:.2f} V")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5762", help="SITL SERIAL1")
    ap.add_argument("--weather", default="data/weather/truth.csv")
    ap.add_argument("--log", default="sim/logs/env_sim.csv")
    ap.add_argument("--state", default="sim/state/world.json")
    ap.add_argument("--fresh", action="store_true", help="ignore the saved state, start at V_START")
    a = ap.parse_args()

    sim = EnvSim(a.conn, WeatherTruth(a.weather), Path(a.log), Path(a.state), fresh=a.fresh)
    sim.connect()
    next_hb = next_pub = next_log = next_save = 0.0
    while True:
        if not sim.read():
            print("env_sim: autopilot link lost")
            sim.t_unix = 0.0                      # wait for the new SITL's SYSTEM_TIME
            next_pub = next_log = next_save = 0.0
            sim.connect()
            continue
        now_wall = time.monotonic()
        if now_wall >= next_hb:
            sim.heartbeat()
            next_hb = now_wall + 1.0
        if sim.t_unix and next_pub - sim.t_unix > PUBLISH_EVERY_S:
            next_pub = next_log = next_save = 0.0   # clock went backwards: restart the schedules
        if sim.t_unix and sim.t_unix >= next_pub:
            sim.step()
            sim.publish()
            next_pub = sim.t_unix + PUBLISH_EVERY_S
            if sim.t_unix >= next_log:
                sim.write_log()
                next_log = sim.t_unix + LOG_EVERY_S
            if sim.t_unix >= next_save:
                sim.save_state()
                next_save = sim.t_unix + SAVE_EVERY_S


if __name__ == "__main__":
    main()
