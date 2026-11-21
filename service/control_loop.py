"""Control loop, step 3: plan once on start, start the mission, then follow the plan on every tick.

On start:
  1. wait for a usable boat state (autopilot time, fresh battery voltage, position)
  2. run the planner from that state (planner.make_plan)
  3. save the plan to service/state/plan.json and queue it for shore
  4. upload the mission waypoints to the autopilot, switch to AUTO, arm

On every tick:
  - set the autopilot speed to the plan's speed for the current hour (HOLD if the plan says 0)
  - switch Starlink on/off according to the mission schedule
  - log the state

Telemetry to shore: 1-minute averages (service/minute_bins.py), the same bins as the plan series.

TEMPORARY: the mission is read from planner/mission.json and the forecast from
data/weather/forecast.csv. Both should arrive from shore (mission upload / forecast pack).

Without a plan (--no-plan, or the planner failed) the boat runs at the mission's speed_ms.

Ticks run on *autopilot* time, so they speed up with SIM_SPEEDUP.

Run (SITL + env sim running; shore server optional):
    python3 service/control_loop.py
    python3 service/control_loop.py --tick 60
"""
import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import planner
from bridge import Bridge
from minute_bins import MinuteBins
from shore_link import ShoreLink

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "service" / "state"


def run(bridge, shore, pilot, tick_s):
    """Two clocks, both autopilot time: telemetry = 1-minute averages (same bins as the plan series),
    control = every tick_s."""
    bins = MinuteBins()
    next_tick = 0.0
    while True:
        state = bridge.snapshot()
        record = bins.add(state)
        if record is not None:
            shore.send(record)
        if state.t_unix and state.t_unix >= next_tick:
            on_tick(state, pilot)
            next_tick = state.t_unix + tick_s
        time.sleep(0.05)   # small wall-clock nap; the tick itself is autopilot time


def on_tick(state, pilot):
    """One control step: apply the plan's setpoints and log what we see."""
    pilot.apply(state)
    print(state.line())


# ---------- autopilot ----------
def start_mission(bridge, mission):
    """Upload the mission waypoints, switch to AUTO and arm. Return True if all three were accepted."""
    s = bridge.snapshot()
    if s.mode == "AUTO":       # each item written to a running AUTO mission restarts it -> leave AUTO first
        bridge.set_mode("MANUAL")
    home = {"lat": s.lat, "lon": s.lon}
    ok = bridge.upload_mission(mission["waypoints"], home)
    print(f"autopilot: mission upload ({len(mission['waypoints'])} waypoints) -> {'ok' if ok else 'FAILED'}")
    if not ok:
        return False
    ok_mode = bridge.set_mode("AUTO")
    ok_arm = bridge.arm()
    print(f"autopilot: AUTO -> {'ok' if ok_mode else 'FAILED'}, arm -> {'ok' if ok_arm else 'FAILED'}")
    return ok_mode and ok_arm


class Pilot:
    """Turns the plan (or the mission, without a plan) into autopilot setpoints.

    Speed comes from plan["hourly"] (the entry for the current hour; the last one after the plan ends).
    Starlink follows the mission schedule (planner.starlink_on).
    Every tick the boat is kept armed and in AUTO (re-armed / re-switched if not; this also retries a
    start that failed, e.g. EKF not ready yet). For now that includes overriding a deliberate disarm.
    Speed is re-sent when it changes, after a mode change, or if the last command was refused; Starlink
    is compared with the measured relay output, so a missed command is retried on the next tick."""

    def __init__(self, bridge, mission, plan=None):
        self.bridge = bridge
        self.mission = mission
        self.plan = plan
        self.speed_cmd = None          # last speed the autopilot accepted

    def setpoints(self, t_unix):
        """-> (speed m/s, starlink on?) wanted at autopilot time t_unix."""
        if self.plan and self.plan["hourly"]:
            past = [h["speed"] for h in self.plan["hourly"] if h["t"] <= t_unix]
            speed = past[-1] if past else self.plan["hourly"][0]["speed"]
        else:
            speed = self.mission["speed_ms"]
        now = datetime.fromtimestamp(t_unix, timezone.utc)
        return float(speed), bool(planner.starlink_on(now, self.mission["starlink"]))

    def apply(self, state):
        speed, starlink = self.setpoints(state.t_unix)
        # Rover refuses DO_CHANGE_SPEED 0, so "stop" = HOLD; any other speed = AUTO at that speed
        want_mode = "HOLD" if speed <= 0 else "AUTO"
        if state.mode != want_mode:
            ok = self.bridge.set_mode(want_mode)
            print(f"autopilot: mode {state.mode} -> {want_mode} {'ok' if ok else 'REFUSED'}")
            self.speed_cmd = None      # entering AUTO resets the speed to WP_SPEED -> send ours again
        if not state.armed:
            ok = self.bridge.arm()
            print(f"autopilot: arm {'ok' if ok else 'REFUSED'}")
        if speed > 0 and speed != self.speed_cmd:
            ok = self.bridge.set_speed(speed)
            print(f"autopilot: speed -> {speed} m/s {'ok' if ok else 'REFUSED'}")
            self.speed_cmd = speed if ok else None
        if starlink != state.starlink_on:
            ok = self.bridge.set_starlink(starlink)
            print(f"autopilot: starlink -> {'on' if starlink else 'off'} {'ok' if ok else 'REFUSED'}")


# ---------- planning ----------
def wait_for_plannable_state(bridge, timeout_s=120):
    """Block until we know time, battery voltage and position (wall-clock timeout)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = bridge.snapshot()
        if s.t_unix and s.battery_fresh() and s.batt_v > 0 and (s.lat, s.lon) != (0.0, 0.0):
            return s
        time.sleep(0.5)
    return None


def save_json(path, obj):
    """Write atomically: a crash mid-write never leaves a half plan on disk."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(obj, indent=1))
    os.replace(tmp, path)


def plan_on_start(bridge, shore, mission, forecast_path):
    print("planner: waiting for time, battery voltage and position...")
    s = wait_for_plannable_state(bridge)
    if s is None:
        print("planner: no usable state after 120 s -> running without a plan")
        return None
    forecast = planner.read_forecast(forecast_path)
    tic = time.monotonic()
    try:
        plan = planner.make_plan(mission, forecast, s.t_unix, s.batt_v, s.lat, s.lon)
    except ValueError as e:                # e.g. deadline already passed, outside the forecast
        print(f"planner: cannot plan ({e}) -> running without a plan")
        return None
    sm = plan["summary"]
    print(f"planner: {plan['plan_id']} by {plan['planner']} in {time.monotonic() - tic:.1f} s wall: "
          f"{plan['route_km']} km, arrival {sm['arrival']}, min {sm['min_v']} V "
          f"(reserve {plan['reserve_v']}), feasible={sm['feasible']}")
    save_json(STATE_DIR / "plan.json", plan)
    shore.send_plan(plan)
    return plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--tick", type=float, default=60.0, help="seconds of autopilot time per tick")
    ap.add_argument("--shore", default="http://127.0.0.1:8000/api/telemetry")
    ap.add_argument("--mission", default=str(ROOT / "planner" / "mission_mini.json"), help="TEMPORARY: local mission")
    ap.add_argument("--forecast", default=str(ROOT / "data" / "weather" / "forecast.csv"),
                    help="TEMPORARY: local forecast")
    ap.add_argument("--no-plan", action="store_true", help="skip planning on start")
    a = ap.parse_args()

    mission = json.loads(Path(a.mission).read_text())

    # Bridge = our only MAVLink connection to the autopilot. start() blocks until the autopilot's
    # heartbeat arrives, asks it to stream the messages we need, and starts the reader thread
    # that keeps bridge.snapshot() up to date.
    bridge = Bridge(a.conn)
    bridge.start()

    # ShoreLink = store-and-forward to the shore server. start() launches the background thread that
    # POSTs queued telemetry (and the latest plan) whenever shore is reachable; it never blocks us.
    shore = ShoreLink(a.shore)
    shore.start()

    # Plan once from the current state (time, battery V, position). None if skipped or it failed.
    plan = None if a.no_plan else plan_on_start(bridge, shore, mission, a.forecast)

    # Wait for GPS fix, EKF position, home and pre-arm checks (BoatState.not_ready()), then give the
    # autopilot the route, switch to AUTO and arm: from here on the boat is moving. If something
    # fails here, Pilot retries AUTO + arm on every tick.
    bridge.wait_until_ready()
    start_mission(bridge, mission)

    # Pilot = turns the plan into speed / Starlink commands; run() calls it on every tick.
    pilot = Pilot(bridge, mission, plan)
    run(bridge, shore, pilot, a.tick)


if __name__ == "__main__":
    main()
