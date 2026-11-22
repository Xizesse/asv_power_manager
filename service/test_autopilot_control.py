"""Try the autopilot commands one by one and show what the autopilot says back.

Steps, in the order the control loop does them:
  0. wait until the vehicle is ready: GPS 3D fix, EKF position estimate, home set, pre-arm checks pass
     (prints what is still missing whenever that changes)
  1. switch to MANUAL if in AUTO (each item written to a running AUTO mission restarts it),
     upload the mission (shore/missions/mission.json waypoints), then read back how many items it holds
  2. set mode AUTO
  3. arm
  4. change speed
then watch the boat state for a while to see if it actually moves.

Autopilot STATUSTEXT messages (its reasons for refusing things) are printed as "autopilot: ...".

Run (SITL running; stop the control loop first, both listen on 14551):
    python3 service/test_autopilot_control.py
    python3 service/test_autopilot_control.py --speed 0.8 --watch 60
    python3 service/test_autopilot_control.py --skip mission     # e.g. mission already uploaded
"""
import argparse
import json
import time
from pathlib import Path

from bridge import Bridge

ROOT = Path(__file__).resolve().parents[1]
STEPS = ["mission", "auto", "arm", "speed"]


def step(name, fn):
    print(f"\n=== {name} ===")
    ok = fn()
    time.sleep(1.0)                      # let STATUSTEXT / the next heartbeat arrive
    s = bridge.snapshot()
    print(f"--> {'OK' if ok else 'FAILED'}   (mode={s.mode}, {'ARMED' if s.armed else 'disarmed'})")
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--mission", default=str(ROOT / "shore" / "missions" / "mission.json"))
    ap.add_argument("--speed", type=float, default=1.0, help="m/s for the speed step")
    ap.add_argument("--skip", nargs="*", default=[], choices=STEPS, help="steps to skip")
    ap.add_argument("--watch", type=float, default=30, help="wall-clock seconds to watch the state after")
    ap.add_argument("--ready-timeout", type=float, default=300, help="wall-clock seconds to wait for readiness")
    a = ap.parse_args()

    bridge = Bridge(a.conn)
    bridge.start()
    print("\n=== waiting for the vehicle to be ready ===")
    s = bridge.wait_until_ready(a.ready_timeout)
    if s is None:
        raise SystemExit(1)
    print("start:", s.line())

    if "mission" not in a.skip:
        if s.mode == "AUTO":
            step("set mode MANUAL (don't rewrite a running mission)", lambda: bridge.set_mode("MANUAL"))
        mission = json.loads(Path(a.mission).read_text())
        home = {"lat": s.lat, "lon": s.lon}
        step(f"upload mission ({len(mission['waypoints'])} waypoints + home)",
             lambda: bridge.upload_mission(mission["waypoints"], home))
        print(f"autopilot now holds {bridge.mission_count()} items "
              f"(expected {len(mission['waypoints']) + 1})")
    if "auto" not in a.skip:
        step("set mode AUTO", lambda: bridge.set_mode("AUTO"))
    if "arm" not in a.skip:
        step("arm", lambda: bridge.arm())
    if "speed" not in a.skip:
        step(f"set speed {a.speed} m/s", lambda: bridge.set_speed(a.speed))

    print(f"\n=== watching for {a.watch:.0f} s ===")
    end = time.monotonic() + a.watch
    while time.monotonic() < end:
        print(bridge.snapshot().line())
        time.sleep(2)
