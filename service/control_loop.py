"""Control loop, step 1: read the boat state on every tick, log it, send it to shore.

Ticks run on *autopilot* time, so they speed up with SIM_SPEEDUP.
Later: decide (planner, supervisor) and call bridge.set_mode/set_speed/set_starlink.

Run (SITL + env sim running; shore server optional):
    python3 service/control_loop.py
    python3 service/control_loop.py --tick 60
"""
import argparse
import time

from bridge import Bridge
from shore_link import ShoreLink


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--tick", type=float, default=60.0, help="seconds of autopilot time per tick")
    ap.add_argument("--shore", default="http://127.0.0.1:8000/api/telemetry")
    a = ap.parse_args()

    bridge = Bridge(a.conn)
    bridge.start()
    shore = ShoreLink(a.shore)
    shore.start()
    run(bridge, shore, a.tick)


if __name__ == "__main__":
    main()
