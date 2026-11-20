"""Control loop, step 2: plan once on start, then read the boat state on every tick, log it, send it to shore.

On start:
  1. wait for a usable boat state (autopilot time, fresh battery voltage, position)
  2. run the planner from that state (planner.make_plan)
  3. save the plan to service/state/plan.json and queue it for shore

TEMPORARY: the mission is read from planner/mission.json and the forecast from
data/weather/forecast.csv. Both should arrive from shore (mission upload / forecast pack).

The plan is NOT applied yet (no speed / Starlink / mission commands to the autopilot).

Ticks run on *autopilot* time, so they speed up with SIM_SPEEDUP.

Run (SITL + env sim running; shore server optional):
    python3 service/control_loop.py
    python3 service/control_loop.py --tick 60
"""
import argparse
import json
import os
import time
from pathlib import Path

import planner
from bridge import Bridge
from shore_link import ShoreLink

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "service" / "state"


def run(bridge, shore, tick_s):
    next_tick = 0.0
    while True:
        state = bridge.snapshot()
        if state.t_unix and state.t_unix >= next_tick:
            on_tick(state, shore)
            next_tick = state.t_unix + tick_s
        time.sleep(0.05)   # small wall-clock nap; the tick itself is autopilot time


def on_tick(state, shore):
    """One control step. For now: log what we see and report it to shore."""
    print(state.line())
    fresh = state.battery_fresh()          # no recent battery message -> send null, never a fake 0
    shore.send({
        "t": state.t_unix,
        "mode": state.mode,
        "armed": state.armed,
        "lat": state.lat, "lon": state.lon,
        "speed": round(state.speed, 2),
        "throttle_pct": round(state.throttle_pct, 1),
        "starlink": state.starlink_on,
        "batt_v": round(state.batt_v, 3) if fresh else None,
        "batt_a": round(state.batt_a, 2) if fresh else None,
        "load_w": round(state.load_v * state.load_a, 1) if fresh else None,
        "solar_w": round(state.solar_v * state.solar_a, 1) if fresh else None,
    })


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


def plan_on_start(bridge, shore, mission_path, forecast_path):
    print("planner: waiting for time, battery voltage and position...")
    s = wait_for_plannable_state(bridge)
    if s is None:
        print("planner: no usable state after 120 s -> running without a plan")
        return None
    mission = json.loads(Path(mission_path).read_text())
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
    ap.add_argument("--mission", default=str(ROOT / "planner" / "mission.json"), help="TEMPORARY: local mission")
    ap.add_argument("--forecast", default=str(ROOT / "data" / "weather" / "forecast.csv"),
                    help="TEMPORARY: local forecast")
    ap.add_argument("--no-plan", action="store_true", help="skip planning on start")
    a = ap.parse_args()

    bridge = Bridge(a.conn)
    bridge.start()
    shore = ShoreLink(a.shore)
    shore.start()
    if not a.no_plan:
        plan_on_start(bridge, shore, a.mission, a.forecast)
    run(bridge, shore, a.tick)


if __name__ == "__main__":
    main()
