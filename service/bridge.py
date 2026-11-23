"""MAVLink bridge: the only part of the service that talks to the autopilot.

- A background thread reads MAVLink and keeps `BoatState` up to date. `start()` never blocks: the
  autopilot is picked up whenever its heartbeat arrives, and again after every gap (reboot).
- `snapshot()` gives the control loop a copy of the latest state.
- `set_mode()`, `set_speed()`, `set_starlink()`, `arm()` send a command and wait for its ack.
- `upload_mission()` replaces the autopilot's mission with a list of waypoints.

Run on its own to watch the state (prints every 5 s of autopilot time):
    python3 service/bridge.py
"""
import argparse
import queue
import threading
import time
from dataclasses import dataclass, replace
from datetime import datetime, timezone

from pymavlink import mavutil

import log

AUTOPILOT_COMP = 1
OWN_SYSID = 200               # the autopilot's "GCS" (MAV_GCS_SYSID 200): our heartbeat keeps its failsafe quiet
OWN_COMP = 191                # MAV_COMP_ID_ONBOARD_COMPUTER
HEARTBEAT_EVERY_S = 1.0       # our own heartbeat; ArduPilot holds after FS_TIMEOUT (5 s) without it
STARLINK_SERVO = 9
HEARTBEAT_GAP_S = 3.0        # heartbeat back after a longer silence = reconnect (re-request streams)
STARLINK_ON_PWM, STARLINK_OFF_PWM = 1900, 1100
STREAMS_HZ = {"SYSTEM_TIME": 1, "GLOBAL_POSITION_INT": 1, "VFR_HUD": 1, "SERVO_OUTPUT_RAW": 1,
              "BATTERY_STATUS": 1,
              # readiness: GPS fix, EKF health, pre-arm checks, home
              "GPS_RAW_INT": 1, "EKF_STATUS_REPORT": 1, "SYS_STATUS": 1, "HOME_POSITION": 0.2,
              "MISSION_CURRENT": 1}   # MISSION_ITEM_REACHED is sent by the autopilot on its own
# EKF must have all of these before the autopilot can navigate (and arm)
EKF_NEEDED = {"attitude": "EKF_ATTITUDE", "horizontal velocity": "EKF_VELOCITY_HORIZ",
              "absolute position": "EKF_POS_HORIZ_ABS", "predicted position": "EKF_PRED_POS_HORIZ_ABS"}


def _enum(name, value):
    """Readable name of a MAVLink enum value, e.g. _enum('MAV_RESULT', 4) -> 'MAV_RESULT_FAILED'."""
    entry = mavutil.mavlink.enums.get(name, {}).get(value)
    return entry.name if entry else f"{name}={value}"


@dataclass
class BoatState:
    t_unix: float = 0.0          # autopilot time (GPS), seconds
    mode: str = "?"
    armed: bool = False
    lat: float = 0.0
    lon: float = 0.0
    speed: float = 0.0           # m/s over ground
    throttle_pwm: int = 0        # output 3
    starlink_pwm: int = 0        # output 9
    last_heartbeat: float = 0.0  # autopilot time of last heartbeat
    # liveness, in WALL-clock time (time.monotonic): autopilot time stops when the autopilot dies,
    # so "how long since we heard from it" can only be measured with our own clock. 0 = never.
    heartbeat_wall: float = 0.0
    battery_wall: float = 0.0
    # BATTERY_STATUS by id: 0 = battery (net, signed), 1 = total load, 2 = solar
    batt_v: float = 0.0
    batt_a: float = 0.0
    load_v: float = 0.0
    load_a: float = 0.0
    solar_v: float = 0.0
    solar_a: float = 0.0
    batt_t: float = 0.0          # autopilot time of the last BATTERY_STATUS (0 = never)
    # readiness
    gps_fix: int = 0             # GPS_RAW_INT.fix_type: 3 = 3D fix
    gps_sats: int = 0
    ekf_flags: int = 0           # EKF_STATUS_REPORT.flags
    prearm_ok: bool = False      # SYS_STATUS: autopilot's own pre-arm checks pass
    home_set: bool = False       # HOME_POSITION received
    # mission progress
    mission_seq: int = -1        # MISSION_CURRENT.seq: item the autopilot is heading to (-1 = unknown)
    reached_seq: int = -1        # MISSION_ITEM_REACHED.seq: last item reached since we connected

    def not_ready(self):
        """What is still missing before we can upload a mission, go AUTO and arm ([] = ready)."""
        missing = []
        if not self.t_unix:
            missing.append("autopilot time")
        if self.gps_fix < 3:
            missing.append(f"GPS 3D fix (fix_type={self.gps_fix}, sats={self.gps_sats})")
        lacking = [name for name, flag in EKF_NEEDED.items()
                   if not self.ekf_flags & getattr(mavutil.mavlink, flag)]
        if self.ekf_flags & mavutil.mavlink.EKF_CONST_POS_MODE:
            lacking.append("not in constant-position mode")
        if lacking:
            missing.append("EKF: " + ", ".join(lacking))
        if not self.home_set:
            missing.append("home position")
        if not self.prearm_ok and not self.armed:   # the flag is only meaningful while disarmed
            missing.append("pre-arm checks")
        return missing

    def heartbeat_age(self):
        """Wall seconds since the last autopilot heartbeat (inf = never)."""
        return time.monotonic() - self.heartbeat_wall if self.heartbeat_wall else float("inf")

    def battery_age(self):
        """Wall seconds since the last BATTERY_STATUS (inf = never)."""
        return time.monotonic() - self.battery_wall if self.battery_wall else float("inf")

    def battery_fresh(self, max_age_s=30.0):
        """True if battery data arrived recently (in autopilot time)."""
        return self.batt_t > 0 and self.t_unix - self.batt_t <= max_age_s

    @property
    def throttle_pct(self):
        """-100..100 %: 1500 us = stopped, 2000 = full ahead, 1000 = full astern."""
        return (self.throttle_pwm - 1500) / 5 if self.throttle_pwm else 0.0

    @property
    def starlink_on(self):
        # 0 = output 9 never commanded since boot -> relay sits at its default, SERVO9_TRIM 1900 = ON.
        # Must match sim/env_sim.py; treating 0 as off meant the Pilot never sent the 'off' command.
        return self.starlink_pwm == 0 or self.starlink_pwm > 1500

    def line(self):
        t = datetime.fromtimestamp(self.t_unix, timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
        return (f"{t}  {self.mode:<8} {'ARMED' if self.armed else 'disarmed':<8} "
                f"{self.lat:.5f},{self.lon:.5f}  {self.speed:4.1f} m/s  "
                f"thr={self.throttle_pct:+4.0f}%  starlink={'on' if self.starlink_on else 'off'}({self.starlink_pwm})  "
                + (f"batt={self.batt_v:5.2f}V {self.batt_a:+5.1f}A  "
                   f"load={self.load_v * self.load_a:5.1f}W  solar={self.solar_v * self.solar_a:5.1f}W"
                   if self.battery_fresh() else "batt=NO DATA"))


class Bridge:
    def __init__(self, conn="udpin:127.0.0.1:14551"):
        self.m = mavutil.mavlink_connection(conn, source_system=OWN_SYSID, source_component=OWN_COMP)
        self._state = BoatState()
        self._lock = threading.Lock()
        self._acks = queue.Queue()
        self._mission_msgs = queue.Queue()     # MISSION_REQUEST(_INT) / MISSION_ACK during an upload

    # ---------- lifecycle ----------
    def start(self):
        """Start the reader thread and return at once (never blocks). The reader picks up the
        autopilot whenever its heartbeat arrives (see _on_autopilot_heartbeat)."""
        log.info("bridge", "listening for the autopilot heartbeat...")
        threading.Thread(target=self._reader, daemon=True, name="mavlink").start()
        threading.Thread(target=self._heartbeat, daemon=True, name="heartbeat").start()

    def _heartbeat(self):
        """Our heartbeat, 1 Hz, on its own thread: the main loop may block for seconds (planning) and
        that must not trip the autopilot's failsafe. So it proves the PROCESS is alive, not the loop.
        Sent only once the autopilot is known (udpin learns where to send from what it receives)."""
        while True:
            if self._state.heartbeat_wall:
                try:
                    self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                              mavutil.mavlink.MAV_STATE_ACTIVE)
                except OSError as e:
                    log.warning("bridge", f"heartbeat send failed ({e})")
            time.sleep(HEARTBEAT_EVERY_S)

    def _on_autopilot_heartbeat(self, src_system, first):
        """First heartbeat, or first after a gap (autopilot rebooted / link back): (re)target it and
        ask for the messages we need. ArduPilot forgets SET_MESSAGE_INTERVAL on reboot, so this is
        repeated every time the heartbeat comes back."""
        self.m.target_system, self.m.target_component = src_system, AUTOPILOT_COMP
        log.info("bridge", f"{'connected to' if first else 'reconnected to'} autopilot "
                           f"(system {src_system}), requesting message streams")
        for name, hz in STREAMS_HZ.items():   # sent directly, not via _command: this runs on the reader
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")   # thread and must not drain the
            self.m.mav.command_long_send(src_system, AUTOPILOT_COMP,        # acks the main loop waits for
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    def snapshot(self):
        """Copy of the latest state (safe to use while the reader keeps updating)."""
        with self._lock:
            return replace(self._state)

    def forget_reached(self):
        """Clear the last MISSION_ITEM_REACHED (e.g. before a new mission starts: an old one must not count)."""
        with self._lock:
            self._state.reached_seq = -1

    def wait_until_ready(self, timeout_s=300):
        """Block until BoatState.not_ready() is empty (wall-clock timeout); print what is missing
        each time that changes. Returns the ready state, or None on timeout."""
        deadline = time.monotonic() + timeout_s
        last = None
        while time.monotonic() < deadline:
            s = self.snapshot()
            missing = s.not_ready()
            if not missing:
                log.info("bridge", "vehicle ready")
                return s
            if missing != last:
                log.info("bridge", "waiting for: " + "; ".join(missing))
                last = missing
            time.sleep(0.5)
        log.warning("bridge", f"vehicle not ready after {timeout_s:.0f} s: {'; '.join(self.snapshot().not_ready())}")
        return None

    # ---------- commands ----------
    def set_mode(self, name):
        mode_id = self.m.mode_mapping()[name.upper()]
        return self._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def set_speed(self, m_s):
        return self._command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, m_s, -1)  # 1 = ground speed

    def set_starlink(self, on):
        pwm = STARLINK_ON_PWM if on else STARLINK_OFF_PWM
        return self._command(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, STARLINK_SERVO, pwm)

    def arm(self, on=True):
        return self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1 if on else 0)

    def set_mission_current(self, seq):
        """Make mission item `seq` the next one. While not in AUTO, AUTO will resume from it (MIS_RESTART 0)."""
        return self._command(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, seq, 0)

    def upload_mission(self, waypoints, home, timeout=10.0):
        """Replace the autopilot mission with `waypoints` ([{lat, lon}, ...]). Return True if accepted.

        MAVLink upload handshake: we send MISSION_COUNT, the autopilot asks for each item by seq
        (MISSION_REQUEST_INT), we answer with MISSION_ITEM_INT, it ends with MISSION_ACK.
        ArduPilot keeps item 0 for home, so `home` ({lat, lon}) goes there and the route starts at 1."""
        items = [home] + list(waypoints)
        while not self._mission_msgs.empty():
            self._mission_msgs.get_nowait()
        self.m.mav.mission_count_send(self.m.target_system, AUTOPILOT_COMP, len(items))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                msg = self._mission_msgs.get(timeout=deadline - time.monotonic())
            except queue.Empty:
                break
            if msg.get_type() == "MISSION_ACK":
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    return True
                log.error("bridge", f"mission rejected: {_enum('MAV_MISSION_RESULT', msg.type)}")
                return False
            if msg.get_type() == "MISSION_COUNT":
                continue
            if msg.seq >= len(items):
                log.error("bridge", f"autopilot asked for mission item {msg.seq}, we only have {len(items)}")
                continue
            log.debug("bridge", f"mission item {msg.seq + 1}/{len(items)} requested")
            wp = items[msg.seq]
            self.m.mav.mission_item_int_send(
                self.m.target_system, AUTOPILOT_COMP, msg.seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,                    # current, autocontinue
                0, 0, 0, 0,              # hold time, accept radius (0 = WP_RADIUS), pass radius, yaw
                int(wp["lat"] * 1e7), int(wp["lon"] * 1e7), 0)
            deadline = time.monotonic() + timeout   # each request restarts the timeout
        log.error("bridge", "mission upload timed out (no request/ack from the autopilot)")
        return False

    def mission_count(self, timeout=5.0):
        """Ask the autopilot how many mission items it holds (including home). None if no answer."""
        while not self._mission_msgs.empty():
            self._mission_msgs.get_nowait()
        self.m.mav.mission_request_list_send(self.m.target_system, AUTOPILOT_COMP)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                msg = self._mission_msgs.get(timeout=deadline - time.monotonic())
            except queue.Empty:
                break
            if msg.get_type() == "MISSION_COUNT":
                # end the download politely so the autopilot doesn't wait for item requests
                self.m.mav.mission_ack_send(self.m.target_system, AUTOPILOT_COMP,
                                            mavutil.mavlink.MAV_MISSION_ACCEPTED)
                return msg.count
        return None

    # ---------- internals ----------
    def _command(self, cmd, p1=0, p2=0, p3=0, p4=0, wait=True, timeout=3.0):
        """Send COMMAND_LONG; return True if the autopilot accepted it."""
        while not self._acks.empty():          # drop stale acks
            self._acks.get_nowait()
        self.m.mav.command_long_send(self.m.target_system, AUTOPILOT_COMP, cmd, 0,
                                     p1, p2, p3, p4, 0, 0, 0)
        if not wait:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                ack = self._acks.get(timeout=deadline - time.monotonic())
            except queue.Empty:
                break
            if ack.command == cmd:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                log.warning("bridge", f"{_enum('MAV_CMD', cmd)} refused: {_enum('MAV_RESULT', ack.result)}")
                return False
        log.warning("bridge", f"{_enum('MAV_CMD', cmd)}: no ack within {timeout} s")
        return False

    def _reader(self):
        while True:
            msg = self.m.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "BATTERY_STATUS":       # from the autopilot or the env sim (any component)
                self._on_battery(msg)
                continue
            if msg.get_srcComponent() != AUTOPILOT_COMP:
                continue
            if t == "COMMAND_ACK":
                self._acks.put(msg)
                continue
            if t == "STATUSTEXT":
                # ArduPilot's own messages, at its severity (0-3 error, 4 warning, else info)
                level = "error" if msg.severity <= 3 else "warning" if msg.severity == 4 else "info"
                getattr(log, level)("ardupilot", msg.text)
                continue
            if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK", "MISSION_COUNT"):
                self._mission_msgs.put(msg)
                continue
            with self._lock:
                s = self._state
                if t == "HEARTBEAT":
                    if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                        continue
                    now = time.monotonic()
                    first = s.heartbeat_wall == 0
                    reconnect = first or now - s.heartbeat_wall > HEARTBEAT_GAP_S
                    s.mode = mavutil.mode_string_v10(msg)
                    s.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    s.last_heartbeat = s.t_unix
                    s.heartbeat_wall = now
                elif t == "SYSTEM_TIME" and msg.time_unix_usec:
                    s.t_unix = msg.time_unix_usec / 1e6
                elif t == "GLOBAL_POSITION_INT":
                    s.lat, s.lon = msg.lat / 1e7, msg.lon / 1e7
                elif t == "VFR_HUD":
                    s.speed = msg.groundspeed
                elif t == "SERVO_OUTPUT_RAW":
                    s.throttle_pwm = msg.servo3_raw
                    s.starlink_pwm = getattr(msg, "servo9_raw", 0)
                elif t == "GPS_RAW_INT":
                    s.gps_fix, s.gps_sats = msg.fix_type, msg.satellites_visible
                elif t == "EKF_STATUS_REPORT":
                    s.ekf_flags = msg.flags
                elif t == "SYS_STATUS":
                    s.prearm_ok = bool(msg.onboard_control_sensors_health
                                       & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
                elif t == "HOME_POSITION":
                    s.home_set = True
                elif t == "MISSION_CURRENT":
                    s.mission_seq = msg.seq
                elif t == "MISSION_ITEM_REACHED":
                    s.reached_seq = msg.seq
            if t == "HEARTBEAT" and reconnect:        # outside the lock: it sends commands
                self._on_autopilot_heartbeat(msg.get_srcSystem(), first)


    def _on_battery(self, msg):
        volts = msg.voltages[0] / 1000 if msg.voltages[0] not in (0, 65535) else 0.0
        amps = msg.current_battery / 100 if msg.current_battery != -1 else 0.0
        with self._lock:
            s = self._state
            s.batt_t = s.t_unix
            s.battery_wall = time.monotonic()
            if msg.id == 0:
                s.batt_v, s.batt_a = volts, amps
            elif msg.id == 1:
                s.load_v, s.load_a = volts, amps
            elif msg.id == 2:
                s.solar_v, s.solar_a = volts, amps


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    a = ap.parse_args()
    b = Bridge(a.conn)
    b.start()
    while True:
        print(b.snapshot().line())
        time.sleep(5)
