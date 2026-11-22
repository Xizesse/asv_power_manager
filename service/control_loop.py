"""Control loop: a small state machine. The mission and the forecast come from shore (over Starlink).

States (Controller.state):
  WAITING_FOR_MISSION  no mission on the boat. Starlink forced on, boat in HOLD, polling shore.
  READY                mission + forecast received and saved (service/state/mission.json, forecast.csv),
                       the plan made at once and sent to shore as a preview (so the operator sees
                       arrival / min V / feasible before pressing start), waiting for shore's start command.
                       Starlink still on, boat in HOLD.
  RUNNING              started: the plan made on receipt is followed as it is (no replan at start;
                       runtime replanning comes later). Mission uploaded, AUTO, plan followed.
  DONE                 all waypoints reached -> HOLD.

Starlink is forced on until shore has acked the plan, then follows the mission's schedule. The planner
only knows the schedule: the few minutes of forced-on before the ack are not in the plan (~1 Wh).

Restart (service or autopilot): the saved mission is read back. Started -> replan over the waypoints not
reached yet and continue (no link needed). Not started -> READY. --fresh forgets the mission, its
forecast, progress and plan -> WAITING_FOR_MISSION.

The mission (from shore) holds only operator inputs: waypoints, speed_ms, reserve_v, arrive_by. The
start time and voltage are the boat's own (autopilot time, measured battery V, when it plans), and the
Starlink schedule is a boat setting (STARLINK_SCHEDULE below), merged in when the mission starts.

Shore -> boat travels in the reply to the boat's own Starlink POST (service/links.py); the boat opens
no port. The mission id is shore's: a new upload (even the same route) starts from waypoint 0.

On every tick (RUNNING):
  - set the autopilot speed to the plan's speed for the current hour (HOLD if the plan says 0)
  - switch Starlink on/off (schedule, or forced on until the plan is acked)
  - log the state

Telemetry to shore: 1-minute averages (service/minute_bins.py), the same bins as the plan series.
Two links (service/links.py): Starlink carries every record, the plan and the boat status, but only
while the terminal is on; Iridium sends a small 5-point message every 5 minutes, always.

Without a plan (--no-plan, or the planner failed) the boat runs at the mission's speed_ms.

Ticks run on *autopilot* time, so they speed up with SIM_SPEEDUP.

Run (SITL + env sim + shore server + both proxies running):
    python3 service/control_loop.py
    python3 service/control_loop.py --tick 60
    python3 service/control_loop.py --fresh     # forget the mission: wait for a new one from shore
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
from progress import Progress
import queue

from links import IridiumLink, StarlinkLink

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "service" / "state"


MISSION_FILE = STATE_DIR / "mission.json"      # {"mission_id", "name", "mission", "received_t", "started"}
FORECAST_FILE = STATE_DIR / "forecast.csv"
PROGRESS_FILE = STATE_DIR / "progress.json"
PLAN_FILE = STATE_DIR / "plan.json"
# Starlink: forced on while there is no mission running (WAITING_FOR_MISSION, READY) and until shore has
# acked the plan; otherwise this schedule: on at at_min past every every_h hours (from first_h UTC) for
# on_min minutes. Copies 15-19 Jul (nb 05). Boat setting, not part of a mission.
STARLINK_SCHEDULE = {"every_h": 2, "first_h": 0, "at_min": 47, "on_min": 8}


def run(bridge, starlink, iridium, ctl, tick_s):
    """Two clocks, both autopilot time: telemetry = 1-minute averages (same bins as the plan series),
    control = every tick_s. Messages from shore and waypoint progress are handled on every loop."""
    bins = MinuteBins()
    next_tick = 0.0
    while True:
        ctl.handle_inbox()
        state = bridge.snapshot()
        if ctl.state in ("RUNNING", "DONE") and state.reached_seq >= 0:   # only our running mission counts
            ctl.progress.on_reached(state.reached_seq, state.t_unix)
        record = bins.add(state)
        if record is not None:
            starlink.send(record)
            iridium.add_bin(record)
        if state.t_unix and state.t_unix >= next_tick:
            ctl.tick(state)
            print(f"[{ctl.state}] " + state.line())
            next_tick = state.t_unix + tick_s
        time.sleep(0.05)   # small wall-clock nap; the tick itself is autopilot time


class Controller:
    """The state machine (see the module docstring). Shore messages arrive on `inbox` (filled by the
    Starlink thread); everything that touches the autopilot happens on the main loop."""

    def __init__(self, bridge, starlink, no_plan=False):
        self.bridge = bridge
        self.starlink = starlink
        self.no_plan = no_plan
        self.inbox = queue.Queue()
        self.saved = None          # contents of MISSION_FILE
        self.progress = None
        self.pilot = None
        self.plan = None
        self.state = "WAITING_FOR_MISSION"
        if MISSION_FILE.exists():
            self.saved = json.loads(MISSION_FILE.read_text())
            self.progress = Progress(PROGRESS_FILE, self.saved["mission"], mission_id_=self.saved["mission_id"])
            self.state = "STARTING" if self.saved.get("started") else "READY"
            print(f"mission: {self.saved['mission_id']} loaded from disk, "
                  f"{'started -> continue' if self.saved.get('started') else 'not started -> READY'}")
        else:
            print("mission: none on the boat -> WAITING_FOR_MISSION (Starlink on)")

    # --- shore side (called from the Starlink thread) ---
    def report(self):
        return {"state": self.state,
                "mission_id": self.saved["mission_id"] if self.saved else None,
                "started": bool(self.saved and self.saved.get("started")),
                "plan_id": self.plan["plan_id"] if self.plan else None}

    def on_reply(self, reply):
        if reply.get("to_boat"):
            self.inbox.put(reply["to_boat"])

    # --- main loop ---
    def handle_inbox(self):
        while True:
            try:
                msg = self.inbox.get_nowait()
            except queue.Empty:
                return
            if "mission" in msg:
                self._receive_mission(msg["mission"])
            if "start" in msg:
                self._receive_start(msg["start"])

    def _receive_mission(self, pkg):
        if self.saved and pkg["mission_id"] == self.saved["mission_id"]:
            return                                   # repeat of what we have
        FORECAST_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = FORECAST_FILE.with_suffix(".tmp")      # forecast first: a saved mission always has its forecast
        tmp.write_text(pkg["forecast_csv"])
        os.replace(tmp, FORECAST_FILE)
        self.saved = {"mission_id": pkg["mission_id"], "name": pkg.get("name"), "mission": pkg["mission"],
                      "received_t": self.bridge.snapshot().t_unix, "started": False}
        save_json(MISSION_FILE, self.saved)
        PLAN_FILE.unlink(missing_ok=True)
        self.progress = Progress(PROGRESS_FILE, pkg["mission"], fresh=True, mission_id_=pkg["mission_id"])
        self.pilot = self.plan = None
        self.state = "READY"
        print(f"mission: received {pkg['mission_id']} ({len(pkg['mission']['waypoints'])} waypoints) "
              f"+ forecast -> READY, waiting for start")

    def _receive_start(self, mission_id):
        if not self.saved or mission_id != self.saved["mission_id"] or self.saved.get("started"):
            return
        self.saved["started"] = True
        save_json(MISSION_FILE, self.saved)
        print(f"mission: start command for {mission_id}")
        self.state = "STARTING"

    def tick(self, state):
        if self.state == "STARTING":                 # plan from now, upload, AUTO, arm
            self._start()
        if self.state == "READY" and self.plan is None and not self.no_plan:
            self._preview()
        if self.state in ("WAITING_FOR_MISSION", "READY"):
            self._hold(state)
        elif self.state in ("RUNNING", "DONE"):
            self.pilot.apply(state)
            if self.progress.done:
                self.state = "DONE"

    def _hold(self, state):
        """No mission running: stay put (HOLD) with Starlink on, so shore can reach us."""
        if state.mode and state.mode != "HOLD":
            ok = self.bridge.set_mode("HOLD")
            print(f"autopilot: mode {state.mode} -> HOLD (no mission running) {'ok' if ok else 'REFUSED'}")
        if not state.starlink_on:
            ok = self.bridge.set_starlink(True)
            print(f"autopilot: starlink -> on (waiting for shore) {'ok' if ok else 'REFUSED'}")

    def _preview(self):
        """Plan as soon as the mission is received (also after a restart in READY). Shown on shore as a
        preview; on start it becomes the plan the boat follows."""
        mission = dict(self.saved["mission"], starlink=STARLINK_SCHEDULE)
        self.plan = plan_on_start(self.bridge, self.starlink, mission, self.progress, FORECAST_FILE,
                                  kind="preview") or {"plan_id": None}   # failed: don't retry every tick

    def _start(self):
        mission = dict(self.saved["mission"], starlink=STARLINK_SCHEDULE)
        if self.plan and self.plan.get("kind") == "preview":
            # No replan (for now): follow the plan made when the mission arrived. Whatever the battery
            # did while we waited for the start command is for runtime replanning (not built yet).
            self.plan = dict(self.plan, kind="active", plan_id=self.plan["plan_id"].removesuffix("-preview"),
                             started_t=self.bridge.snapshot().t_unix)
            save_json(PLAN_FILE, self.plan)
            self.starlink.send_plan(self.plan)       # shore: no longer "preview"; Starlink on until acked
            print(f"planner: following {self.plan['plan_id']} (made when the mission arrived)")
        elif not self.no_plan:
            # no preview in memory (service restarted after the start) or it failed -> plan now
            self.plan = plan_on_start(self.bridge, self.starlink, mission, self.progress, FORECAST_FILE)
        else:
            self.plan = None
        self.bridge.wait_until_ready()
        self.bridge.forget_reached()
        start_mission(self.bridge, mission, self.progress)
        self.pilot = Pilot(self.bridge, mission, self.progress, self.plan,
                           force_starlink=self._starlink_forced)
        self.state = "DONE" if self.progress.done else "RUNNING"

    def _starlink_forced(self):
        """Forced on until shore has the plan."""
        return self.plan is not None and self.starlink.plan_acked != self.plan["plan_id"]


# ---------- autopilot ----------
def start_mission(bridge, mission, progress):
    """Upload the full mission, continue from the next unreached waypoint, switch to AUTO and arm.
    Return True if all steps were accepted.

    Always uploads (a few items, < 1 s) instead of trusting what the autopilot holds: simpler, and the
    autopilot's mission is then known to be ours. The current item must be set again anyway, because
    ArduPilot resets to the first waypoint on reboot (AP_Mission::init -> reset)."""
    if progress.done:
        ok = bridge.set_mode("HOLD")
        print(f"autopilot: all {progress.n_wp} waypoints already reached -> HOLD {'ok' if ok else 'REFUSED'}")
        return False
    s = bridge.snapshot()
    if s.mode == "AUTO":       # each item written to a running AUTO mission restarts it -> leave AUTO first
        bridge.set_mode("MANUAL")
    home = {"lat": s.lat, "lon": s.lon}
    ok = bridge.upload_mission(mission["waypoints"], home)
    print(f"autopilot: mission upload ({len(mission['waypoints'])} waypoints) -> {'ok' if ok else 'FAILED'}")
    if not ok:
        return False
    seq = progress.next_wp + 1                 # seq 0 = home
    ok = bridge.set_mission_current(seq)
    print(f"autopilot: continue from waypoint {progress.next_wp} (seq {seq}) -> {'ok' if ok else 'FAILED'}")
    if not ok:
        return False
    ok_mode = bridge.set_mode("AUTO")
    ok_arm = bridge.arm()
    print(f"autopilot: AUTO -> {'ok' if ok_mode else 'FAILED'}, arm -> {'ok' if ok_arm else 'FAILED'}")
    return ok_mode and ok_arm


class Pilot:
    """Turns the plan (or the mission, without a plan) into autopilot setpoints.

    Speed comes from plan["hourly"] (the entry for the current hour; the last one after the plan ends).
    Starlink follows the mission schedule (planner.starlink_on), or is forced on (force_starlink).
    Every tick the boat is kept armed and in AUTO (re-armed / re-switched if not; this also retries a
    start that failed, e.g. EKF not ready yet). For now that includes overriding a deliberate disarm.
    Speed is re-sent when it changes, after a mode change, or if the last command was refused; Starlink
    is compared with the measured relay output, so a missed command is retried on the next tick."""

    def __init__(self, bridge, mission, progress, plan=None, force_starlink=lambda: False):
        self.bridge = bridge
        self.force_starlink = force_starlink   # -> True: Starlink on whatever the schedule says
        self.mission = mission
        self.progress = progress
        self.plan = plan
        self.speed_cmd = None          # last speed the autopilot accepted

    def setpoints(self, t_unix):
        """-> (speed m/s, starlink on?) wanted at autopilot time t_unix. Speed 0 once all waypoints are done."""
        if self.progress.done:
            speed = 0.0
        elif self.plan and self.plan["hourly"]:
            past = [h["speed"] for h in self.plan["hourly"] if h["t"] <= t_unix]
            speed = past[-1] if past else self.plan["hourly"][0]["speed"]
        else:
            speed = self.mission["speed_ms"]
        now = datetime.fromtimestamp(t_unix, timezone.utc)
        starlink = self.force_starlink() or bool(planner.starlink_on(now, self.mission["starlink"]))
        return float(speed), starlink

    def apply(self, state):
        # 1. vehicle not ready (e.g. autopilot just rebooted, EKF/GPS starting) -> touch nothing, wait
        missing = state.not_ready()
        if missing:
            print("pilot: waiting for: " + "; ".join(missing))
            return
        # 2. autopilot behind our progress (it reset to the first waypoint after a reboot) -> start again
        #    from the next unreached waypoint (upload, set current, AUTO, arm)
        expected = self.progress.next_wp + 1           # seq 0 = home
        if not self.progress.done and 0 <= state.mission_seq < expected:
            print(f"pilot: autopilot at mission item {state.mission_seq}, expected {expected} -> restarting mission")
            start_mission(self.bridge, self.mission, self.progress)
            self.speed_cmd = None                      # entering AUTO resets the speed -> send ours again
            return
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


def plan_on_start(bridge, starlink, mission, progress, forecast_path, kind="active"):
    """Plan from the current state over the waypoints not reached yet (after a restart: a replan).
    kind: "preview" (mission received, not started: shown on shore, never followed) or "active"."""
    if progress.done:
        print("planner: all waypoints reached -> no plan")
        return None
    remaining = dict(mission, waypoints=mission["waypoints"][progress.next_wp:])
    print(f"planner: {len(remaining['waypoints'])}/{progress.n_wp} waypoints left "
          f"(from waypoint {progress.next_wp}); waiting for time, battery voltage and position...")
    s = wait_for_plannable_state(bridge)
    if s is None:
        print("planner: no usable state after 120 s -> running without a plan")
        return None
    forecast = planner.read_forecast(forecast_path)
    tic = time.monotonic()
    try:
        plan = planner.make_plan(remaining, forecast, s.t_unix, s.batt_v, s.lat, s.lon)
    except ValueError as e:                # e.g. deadline already passed, outside the forecast
        print(f"planner: cannot plan ({e}) -> running without a plan")
        return None
    plan["kind"] = kind
    if kind == "preview":                  # never the same id as the active plan made in the same minute
        plan["plan_id"] += "-preview"
    sm = plan["summary"]
    print(f"planner: {kind} {plan['plan_id']} by {plan['planner']} in {time.monotonic() - tic:.1f} s wall: "
          f"{plan['route_km']} km, arrival {sm['arrival']}, min {sm['min_v']} V "
          f"(reserve {plan['reserve_v']}), feasible={sm['feasible']}")
    save_json(PLAN_FILE, plan)
    starlink.send_plan(plan)
    return plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--tick", type=float, default=60.0, help="seconds of autopilot time per tick")
    ap.add_argument("--starlink-url", default="http://127.0.0.1:8001", help="Starlink proxy")
    ap.add_argument("--iridium-url", default="http://127.0.0.1:8002", help="Iridium proxy")
    ap.add_argument("--no-plan", action="store_true", help="skip planning on start")
    ap.add_argument("--fresh", action="store_true",
                    help="forget the saved mission, forecast, progress and plan: wait for shore")
    a = ap.parse_args()

    if a.fresh:
        for f in (MISSION_FILE, FORECAST_FILE, PROGRESS_FILE, PLAN_FILE):
            f.unlink(missing_ok=True)
        print("mission: --fresh -> saved mission, forecast, progress and plan deleted")

    # Bridge = our only MAVLink connection to the autopilot. start() blocks until the autopilot's
    # heartbeat arrives, asks it to stream the messages we need, and starts the reader thread
    # that keeps bridge.snapshot() up to date.
    bridge = Bridge(a.conn)
    bridge.start()

    # Links = store-and-forward to shore, each with its own background thread; they never block us.
    # Starlink only talks while the terminal is on (measured relay output); its replies carry shore's
    # messages to the Controller.
    ctl = None
    starlink = StarlinkLink(a.starlink_url, is_on=lambda: bridge.snapshot().starlink_on,
                            report=lambda: ctl.report() if ctl else {},
                            on_reply=lambda reply: ctl and ctl.on_reply(reply))
    iridium = IridiumLink(a.iridium_url)
    ctl = Controller(bridge, starlink, no_plan=a.no_plan)
    starlink.start()
    iridium.start()

    run(bridge, starlink, iridium, ctl, a.tick)


if __name__ == "__main__":
    main()
