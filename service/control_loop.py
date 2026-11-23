"""Control loop: a small state machine. The mission and the forecast come from shore (over Starlink).

States (Controller.state):
  CONNECTING           service just started, no autopilot heartbeat yet. The links already run (shore
                       sees CONNECTING); shore commands wait in the inbox. First heartbeat -> the boot
                       decision: saved + started -> STARTING, saved -> PLANNING, nothing -> WAITING.
                       (A heartbeat lost LATER keeps the state: the pilot waits, health warns.)
  WAITING_FOR_MISSION  no mission on the boat. Starlink forced on, boat in HOLD, polling shore.
  PLANNING             mission + forecast received and saved; the preview plan is being made (a few
                       seconds; HOLD, Starlink on). Then READY. A restart with a saved, not started
                       mission comes back here (the preview is not kept in memory).
  READY                mission + forecast received and saved (service/state/mission.json, forecast.csv),
                       the plan made at once and sent to shore as a preview (so the operator sees
                       arrival / min V / feasible before pressing start), waiting for shore's start command.
                       Starlink still on, boat in HOLD.
  RUNNING              started: the plan made on receipt is followed as it is (no replan at start;
                       runtime replanning comes later). Mission uploaded, AUTO, plan followed.
  DONE                 all waypoints reached -> HOLD, Starlink on (shore can send the next mission).

Starlink is forced on until shore has acked the plan, then follows the mission's schedule. The planner
only knows the schedule: the few minutes of forced-on before the ack are not in the plan (~1 Wh).

Shore commands (the same inbox, whichever link brought them; repeats are harmless, all keyed by mission id):
  mission  (Starlink only: ~110 KB with the forecast) -> PLANNING -> READY
  wake     (Iridium: "a mission is waiting") -> Starlink forced on until that mission is received
  start    (either link) -> STARTING
  abort    (either link) -> mission, forecast, progress and plan deleted -> WAITING_FOR_MISSION
           (HOLD, Starlink on). Ignored if the id is not the mission the boat holds.

Restart (service or autopilot): the saved mission is read back. Started -> replan over the waypoints not
reached yet and continue (no link needed). Not started -> PLANNING -> READY. --fresh forgets the mission, its
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

Telemetry to shore: 10 s averages (service/minute_bins.py, BIN_S; 60 s = the plan series' step at sea).
Two links (service/links.py): Starlink carries every record, the plan and the boat status, but only
while the terminal is on; Iridium sends a small message + the boat status every 10 s,
always. Both read shore's commands from the reply.

Without a plan (--no-plan, or the planner failed) the boat runs at the mission's speed_ms.

Ticks run on *autopilot* time, so they speed up with SIM_SPEEDUP.

Logging (service/log.py): one "component: message" line per event, with a level; under systemd the
level goes to the journal (journalctl -p warning = only what failed). Health (autopilot heartbeat,
battery data) is logged on change + a summary every SUMMARY_EVERY_S. --verbose adds the per-tick line.

Run (SITL + env sim + shore server + both proxies running):
    python3 service/control_loop.py
    python3 service/control_loop.py --tick 60
    python3 service/control_loop.py --fresh     # forget the mission: wait for a new one from shore
"""
import argparse
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import planner
from bridge import Bridge
from minute_bins import MinuteBins
from progress import Progress
import queue

import log

from links import IridiumLink, StarlinkLink

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "service" / "state"


MISSION_FILE = STATE_DIR / "mission.json"      # {"mission_id", "name", "mission", "received_t", "started"}
FORECAST_FILE = STATE_DIR / "forecast.csv"
PROGRESS_FILE = STATE_DIR / "progress.json"
PLAN_FILE = STATE_DIR / "plan.json"               # the decision: summary + hourly speeds (what the boat follows)
PREDICTION_FILE = STATE_DIR / "prediction.json"   # what the plan expects per minute: battery V, speed, load
MISSION_FILES = (MISSION_FILE, FORECAST_FILE, PROGRESS_FILE, PLAN_FILE, PREDICTION_FILE)  # --fresh / abort
# Starlink: forced on while there is no mission running (WAITING_FOR_MISSION, PLANNING, READY) and until shore has
# acked the plan; otherwise this schedule: on at at_min past every every_h hours (from first_h UTC) for
# on_min minutes. Copies 15-19 Jul (nb 05). Boat setting, not part of a mission.
STARLINK_SCHEDULE = {"every_h": 2, "first_h": 0, "at_min": 47, "on_min": 8}
# Health (wall-clock seconds: autopilot time stops when the autopilot dies)
HEARTBEAT_LOST_S = 5.0        # no autopilot HEARTBEAT for this long -> LOST
BATTERY_STALE_S = 15.0        # no BATTERY_STATUS for this long -> stale
SUMMARY_EVERY_S = 60.0        # one health summary line in the journal


def run(bridge, starlink, iridium, ctl, tick_s):
    """Two clocks, both autopilot time: telemetry = BIN_S averages (10 s for the demo),
    control = every tick_s. Messages from shore and waypoint progress are handled on every loop.
    Health (autopilot + battery data alive?) is checked on every loop, on wall-clock time."""
    bins = MinuteBins()
    health = Health(starlink, iridium)
    next_tick = 0.0
    while True:
        ctl.handle_inbox()
        state = bridge.snapshot()
        health.check(state, ctl)
        if ctl.state == "CONNECTING":
            if state.heartbeat_age() >= HEARTBEAT_LOST_S:
                time.sleep(0.2)                   # nothing to control yet; inbox waits
                continue
            ctl.connected()
        if ctl.state in ("RUNNING", "DONE") and state.reached_seq >= 0:   # only our running mission counts
            ctl.progress.on_reached(state.reached_seq, state.t_unix)
        record = bins.add(state)
        if record is not None:
            starlink.send(record)
            iridium.add_bin(record)
        if state.t_unix and state.t_unix >= next_tick:
            ctl.tick(state)
            log.debug("tick", f"[{ctl.state}] " + state.line())
            next_tick = state.t_unix + tick_s
        time.sleep(0.05)   # small wall-clock nap; the tick itself is autopilot time


class Health:
    """Writes down what failed, and when it came back. Logs only CHANGES (OK -> LOST -> back), plus a
    summary line every SUMMARY_EVERY_S, so a quiet journal means "all fine", not "is it running?".

    Liveness is measured on wall-clock time (BoatState.heartbeat_age / battery_age): when the autopilot
    dies its clock stops, so an autopilot-time age would stay 0 forever. Links log their own DOWN/UP
    (service/links.py); here they only appear in the summary."""

    def __init__(self, starlink, iridium):
        self.starlink, self.iridium = starlink, iridium
        self.autopilot_ok = None       # None = not heard yet since start
        self.battery_ok = None
        self.lost_at = {}              # name -> time.monotonic() when it was lost
        self.started = time.monotonic()
        self.next_summary = time.monotonic() + SUMMARY_EVERY_S

    def _change(self, name, ok, lost_msg, back_msg):
        now = time.monotonic()
        if ok:
            log.info("health", f"{back_msg} after {now - self.lost_at.pop(name, now):.0f} s")
        else:
            self.lost_at[name] = now
            log.warning("health", lost_msg)

    def check(self, s, ctl):
        hb_ok = s.heartbeat_age() < HEARTBEAT_LOST_S
        if self.autopilot_ok is None and not hb_ok:       # since start: warn once if it takes long
            if time.monotonic() - self.started >= HEARTBEAT_LOST_S:
                log.warning("health", f"no autopilot heartbeat yet ({HEARTBEAT_LOST_S:.0f} s since start)")
                self.lost_at["autopilot"] = self.started
                self.autopilot_ok = False
        elif hb_ok != self.autopilot_ok:
            if self.autopilot_ok is not None:
                self._change("autopilot", hb_ok,
                             f"autopilot heartbeat LOST (none for {HEARTBEAT_LOST_S:.0f} s)",
                             f"autopilot heartbeat back (mode {s.mode}, {'armed' if s.armed else 'disarmed'})")
            self.autopilot_ok = hb_ok
        bat_ok = s.battery_age() < BATTERY_STALE_S
        if self.battery_ok is None and not bat_ok:         # since start: warn once if it takes long
            if time.monotonic() - self.started >= BATTERY_STALE_S:
                log.warning("health", f"no battery data yet ({BATTERY_STALE_S:.0f} s since start)")
                self.lost_at["battery"] = self.started
                self.battery_ok = False
        elif bat_ok != self.battery_ok:
            if self.battery_ok is not None:
                self._change("battery", bat_ok,
                             f"battery data STALE (no BATTERY_STATUS for {BATTERY_STALE_S:.0f} s)",
                             f"battery data back ({s.batt_v:.2f} V)")
            self.battery_ok = bat_ok
        if time.monotonic() >= self.next_summary:
            self.next_summary = time.monotonic() + SUMMARY_EVERY_S
            self.summary(s, ctl)

    def summary(self, s, ctl):
        ap = (f"autopilot {s.mode} {'armed' if s.armed else 'disarmed'}" if self.autopilot_ok
              else f"autopilot LOST {s.heartbeat_age():.0f} s")
        bat = f"{s.batt_v:.2f} V" if self.battery_ok else "battery NO DATA"
        wp = f"wp {ctl.progress.next_wp}/{ctl.progress.n_wp}" if ctl.progress else "no mission"
        log.info("health", f"{ctl.state} · {ap} · {bat} · {s.speed:.1f} m/s · {wp} · "
                           f"starlink {self.starlink.status()} · iridium {self.iridium.status()}")


class Controller:
    """The state machine (see the module docstring). Shore messages arrive on `inbox` (filled by the
    Starlink thread); everything that touches the autopilot happens on the main loop."""

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new):
        old = getattr(self, "_state", None)
        if new != old:
            log.info("mission", f"state {old} -> {new}" if old else f"state {new}")
        self._state = new

    def __init__(self, bridge, starlink, no_plan=False):
        self.bridge = bridge
        self.starlink = starlink
        self.no_plan = no_plan
        self.inbox = queue.Queue()
        self.saved = None          # contents of MISSION_FILE
        self.progress = None
        self.pilot = None
        self.plan = None
        self.prediction = None
        self.wake_for = None       # mission id shore said is waiting (Iridium "wake"): Starlink on until we have it
        if MISSION_FILE.exists():
            self.saved = json.loads(MISSION_FILE.read_text())
            log.info("mission", f"{self.saved.get('name')} ({self.saved['mission_id']}) loaded from disk, "
                                f"{'started -> continue' if self.saved.get('started') else 'not started'}")
            self.progress = Progress(PROGRESS_FILE, self.saved["mission"], mission_id_=self.saved["mission_id"])
            self.boot_state = "STARTING" if self.saved.get("started") else "PLANNING"
        else:
            log.info("mission", "none on the boat")
            self.boot_state = "WAITING_FOR_MISSION"
        self.state = "CONNECTING"

    def connected(self):
        """First autopilot heartbeat since the service started: leave CONNECTING."""
        self.state = self.boot_state

    # --- shore side (called from the Starlink thread) ---
    def report(self):
        """Boat status + health, sent with every contact on both links (Iridium: every IRIDIUM_EVERY_S, always)."""
        s = self.bridge.snapshot()
        ap_ok = s.heartbeat_age() < HEARTBEAT_LOST_S
        return {"state": self.state,
                "mission_id": self.saved["mission_id"] if self.saved else None,
                "name": self.saved.get("name") if self.saved else None,
                "started": bool(self.saved and self.saved.get("started")),
                "plan_id": self.plan["plan_id"] if self.plan else None,
                "next_wp": self.progress.next_wp if self.progress else None,
                "autopilot": "ok" if ap_ok else "lost",
                "autopilot_lost_s": None if ap_ok else (None if s.heartbeat_wall == 0
                                                        else round(s.heartbeat_age())),
                "mode": s.mode if ap_ok else None,
                "batt_v": round(s.batt_v, 2) if s.battery_age() < BATTERY_STALE_S else None,
                "lat": round(s.lat, 5) if s.lat else None, "lon": round(s.lon, 5) if s.lon else None}

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
            if "abort" in msg:
                self._receive_abort(msg["abort"])
            if "mission" in msg:
                self._receive_mission(msg["mission"])
            if "wake" in msg:
                self._receive_wake(msg["wake"])
            if "start" in msg:
                self._receive_start(msg["start"])

    def _receive_wake(self, mission_id):
        if (self.saved and self.saved["mission_id"] == mission_id) or self.wake_for == mission_id:
            return                                   # already have it / already waiting for it
        self.wake_for = mission_id
        log.info("mission", f"shore has {mission_id} waiting (wake via Iridium) -> Starlink on until received")
        if self.bridge.snapshot().heartbeat_age() >= HEARTBEAT_LOST_S:
            log.warning("mission", "shore wants Starlink on, but there is no autopilot to switch it "
                                   "(stays requested: switched on when the autopilot is back)")

    def _receive_abort(self, mission_id):
        if not self.saved or self.saved["mission_id"] != mission_id:
            return                                   # not ours (already aborted, or a newer mission)
        for f in MISSION_FILES:
            f.unlink(missing_ok=True)
        self.starlink.drop_plan()                    # an unsent plan of the aborted mission must not reach shore
        self.bridge.forget_reached()
        self.saved = self.progress = self.pilot = self.plan = self.prediction = None
        log.warning("mission", f"ABORT {mission_id} from shore -> mission, forecast, progress and plan deleted")
        self.state = "WAITING_FOR_MISSION"

    def _receive_mission(self, pkg):
        if self.wake_for == pkg["mission_id"]:
            self.wake_for = None
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
        PREDICTION_FILE.unlink(missing_ok=True)
        self.progress = Progress(PROGRESS_FILE, pkg["mission"], fresh=True, mission_id_=pkg["mission_id"])
        self.pilot = self.plan = self.prediction = None
        log.info("mission", f"received {pkg.get('name')} ({pkg['mission_id']}, "
                            f"{len(pkg['mission']['waypoints'])} waypoints) + forecast")
        self.state = "PLANNING"

    def _receive_start(self, mission_id):
        if not self.saved or mission_id != self.saved["mission_id"] or self.saved.get("started"):
            return
        self.saved["started"] = True
        save_json(MISSION_FILE, self.saved)
        log.info("mission", f"start command for {mission_id}")
        self.state = "STARTING"

    def tick(self, state):
        if self.state == "STARTING":                 # plan from now, upload, AUTO, arm
            self._start()
        if self.state in ("WAITING_FOR_MISSION", "PLANNING", "READY"):
            self._hold(state)                        # first: HOLD + Starlink on while we plan
        if self.state == "PLANNING":
            if not self.no_plan:
                self._preview()                      # blocks 2-6 s (shore sees PLANNING meanwhile)
            self.state = "READY"
        elif self.state in ("RUNNING", "DONE"):
            self.pilot.apply(state)
            if self.progress.done:
                self.state = "DONE"

    def _hold(self, state):
        """No mission running: stay put (HOLD) with Starlink on, so shore can reach us."""
        if state.mode and state.mode != "HOLD":
            ok = self.bridge.set_mode("HOLD")
            log.result(ok, "pilot", f"mode {state.mode} -> HOLD (no mission running)")
        if not state.starlink_on:
            ok = self.bridge.set_starlink(True)
            log.result(ok, "pilot", "starlink -> on (waiting for shore)")

    def _preview(self):
        """Plan as soon as the mission is received (PLANNING state, also after a restart). Shown on shore as a
        preview; on start it becomes the plan the boat follows."""
        mission = dict(self.saved["mission"], starlink=STARLINK_SCHEDULE)
        self.plan, self.prediction = plan_on_start(self.bridge, self.starlink, mission, self.progress,
                                                   FORECAST_FILE, kind="preview")
        self.plan = self.plan or {"plan_id": None}     # failed: don't retry every tick

    def _start(self):
        mission = dict(self.saved["mission"], starlink=STARLINK_SCHEDULE)
        if self.plan and self.plan.get("kind") == "preview":
            # No replan (for now): follow the plan made when the mission arrived. Whatever the battery
            # did while we waited for the start command is for runtime replanning (not built yet).
            plan_id = self.plan["plan_id"].removesuffix("-preview")
            self.plan = dict(self.plan, kind="active", plan_id=plan_id, started_t=self.bridge.snapshot().t_unix)
            self.prediction = dict(self.prediction, plan_id=plan_id)
            save_json(PLAN_FILE, self.plan)
            save_json(PREDICTION_FILE, self.prediction)
            self.starlink.send_plan(self.plan, self.prediction)   # shore: no longer "preview"; Starlink on until acked
            log.info("planner", f"following {self.plan['plan_id']} (made when the mission arrived)")
        elif not self.no_plan:
            # no preview in memory (service restarted after the start) or it failed -> plan now
            self.plan, self.prediction = plan_on_start(self.bridge, self.starlink, mission, self.progress,
                                                       FORECAST_FILE)
        else:
            self.plan = self.prediction = None
        self.bridge.wait_until_ready()
        self.bridge.forget_reached()
        start_mission(self.bridge, mission, self.progress)
        self.pilot = Pilot(self.bridge, mission, self.progress, self.plan,
                           force_starlink=self._starlink_forced)
        self.state = "DONE" if self.progress.done else "RUNNING"

    def _starlink_forced(self):
        """Forced on until shore has the plan, or while shore has a new mission waiting for us (wake)."""
        if self.wake_for is not None:
            return True
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
        log.result(ok, "pilot", f"all {progress.n_wp} waypoints already reached -> HOLD")
        return False
    s = bridge.snapshot()
    if s.mode == "AUTO":       # each item written to a running AUTO mission restarts it -> leave AUTO first
        bridge.set_mode("MANUAL")
    home = {"lat": s.lat, "lon": s.lon}
    ok = bridge.upload_mission(mission["waypoints"], home)
    log.result(ok, "pilot", f"mission upload ({len(mission['waypoints'])} waypoints)", fail_level="error")
    if not ok:
        return False
    seq = progress.next_wp + 1                 # seq 0 = home
    ok = bridge.set_mission_current(seq)
    log.result(ok, "pilot", f"continue from waypoint {progress.next_wp} (seq {seq})", fail_level="error")
    if not ok:
        return False
    ok_mode = bridge.set_mode("AUTO")
    ok_arm = bridge.arm()
    (log.info if ok_mode and ok_arm else log.warning)(
        "pilot", f"AUTO {'ok' if ok_mode else 'REFUSED'}, arm {'ok' if ok_arm else 'REFUSED'}"
                 + ("" if ok_mode and ok_arm else " (retried next tick)"))
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
        self.missing = None            # last "waiting for" list (logged only when it changes)

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
        # DONE (all waypoints reached): Starlink stays on, so shore can reach the boat for the next mission
        starlink = (self.progress.done or self.force_starlink()
                    or bool(planner.starlink_on(now, self.mission["starlink"])))
        return float(speed), starlink

    def apply(self, state):
        # 1. vehicle not ready (e.g. autopilot just rebooted, EKF/GPS starting) -> touch nothing, wait
        missing = state.not_ready()
        if missing != self.missing and (missing or self.missing):
            log.info("pilot", ("waiting for: " + "; ".join(missing)) if missing else "vehicle ready")
        self.missing = missing
        if missing:
            return
        # 2. autopilot behind our progress (it reset to the first waypoint after a reboot) -> start again
        #    from the next unreached waypoint (upload, set current, AUTO, arm)
        expected = self.progress.next_wp + 1           # seq 0 = home
        if not self.progress.done and 0 <= state.mission_seq < expected:
            log.warning("pilot", f"autopilot at mission item {state.mission_seq}, expected {expected} "
                                 f"(rebooted?) -> restarting the mission from waypoint {self.progress.next_wp}")
            start_mission(self.bridge, self.mission, self.progress)
            self.speed_cmd = None                      # entering AUTO resets the speed -> send ours again
            return
        speed, starlink = self.setpoints(state.t_unix)
        # Rover refuses DO_CHANGE_SPEED 0, so "stop" = HOLD; any other speed = AUTO at that speed
        want_mode = "HOLD" if speed <= 0 else "AUTO"
        if state.mode != want_mode:
            ok = self.bridge.set_mode(want_mode)
            log.result(ok, "pilot", f"mode {state.mode} -> {want_mode}")
            self.speed_cmd = None      # entering AUTO resets the speed to WP_SPEED -> send ours again
        if not state.armed:
            ok = self.bridge.arm()
            log.result(ok, "pilot", "arm")
        if speed > 0 and speed != self.speed_cmd:
            ok = self.bridge.set_speed(speed)
            log.result(ok, "pilot", f"speed {self.speed_cmd if self.speed_cmd is not None else '-'} -> {speed} m/s")
            self.speed_cmd = speed if ok else None
        if starlink != state.starlink_on:
            ok = self.bridge.set_starlink(starlink)
            log.result(ok, "pilot", f"starlink -> {'on' if starlink else 'off'}"
                                    + (" (mission done)" if starlink and self.progress.done
                                       else " (forced: shore)" if starlink and self.force_starlink() else ""))


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
    kind: "preview" (mission received, not started: shown on shore, never followed) or "active".
    Returns (plan, prediction), or (None, None) if there is nothing to plan or it failed."""
    if progress.done:
        log.info("planner", "all waypoints reached -> no plan")
        return None, None
    remaining = dict(mission, waypoints=mission["waypoints"][progress.next_wp:])
    log.info("planner", f"{len(remaining['waypoints'])}/{progress.n_wp} waypoints left "
          f"(from waypoint {progress.next_wp}); waiting for time, battery voltage and position...")
    s = wait_for_plannable_state(bridge)
    if s is None:
        log.error("planner", "no usable state after 120 s -> running without a plan")
        return None, None
    forecast = planner.read_forecast(forecast_path)
    tic = time.monotonic()
    try:
        plan, prediction = planner.make_plan(remaining, forecast, s.t_unix, s.batt_v, s.lat, s.lon)
    except ValueError as e:                # e.g. deadline already passed, outside the forecast
        log.error("planner", f"cannot plan ({e}) -> running without a plan")
        return None, None
    plan["kind"] = kind
    if kind == "preview":                  # never the same id as the active plan made in the same minute
        plan["plan_id"] += "-preview"
        prediction["plan_id"] = plan["plan_id"]
    sm = plan["summary"]
    (log.info if sm["feasible"] else log.warning)(
        "planner", f"{kind} {plan['plan_id']} by {plan['planner']} in {time.monotonic() - tic:.1f} s wall: "
          f"{plan['route_km']} km, arrival {sm['arrival']}, min {sm['min_v']} V "
          f"(reserve {plan['reserve_v']}), feasible={sm['feasible']}")
    save_json(PLAN_FILE, plan)
    save_json(PREDICTION_FILE, prediction)
    starlink.send_plan(plan, prediction)
    return plan, prediction


def version():
    """git describe of the code that is running (tag + commits + dirty), or 'unknown'."""
    try:
        return subprocess.run(["git", "-C", str(ROOT), "describe", "--tags", "--always", "--dirty"],
                              capture_output=True, text=True, timeout=5).stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--tick", type=float, default=5.0,
                    help="seconds of autopilot time per control tick (5 for the demo; 60 is plenty at sea)")
    ap.add_argument("--starlink-url", default="http://127.0.0.1:8001", help="Starlink proxy")
    ap.add_argument("--iridium-url", default="http://127.0.0.1:8002", help="Iridium proxy")
    ap.add_argument("--no-plan", action="store_true", help="skip planning on start")
    ap.add_argument("--fresh", action="store_true",
                    help="forget the saved mission, forecast, progress and plan: wait for shore")
    ap.add_argument("--verbose", action="store_true", help="also log debug lines (state every tick)")
    a = ap.parse_args()
    log.verbose = a.verbose
    log.info("service", f"start, version {version()}, autopilot {a.conn}, tick {a.tick:.0f} s, "
                        f"state in {STATE_DIR}")

    if a.fresh:
        for f in MISSION_FILES:
            f.unlink(missing_ok=True)
        log.info("mission", "--fresh -> saved mission, forecast, progress, plan and prediction deleted")

    # Bridge = our only MAVLink connection to the autopilot. start() returns at once: its reader
    # thread keeps bridge.snapshot() up to date and picks up the autopilot whenever it appears.
    # Until then the Controller sits in CONNECTING (the links already report it to shore).
    bridge = Bridge(a.conn)
    bridge.start()

    # Links = store-and-forward to shore, each with its own background thread; they never block us.
    # Starlink only talks while the terminal is on (measured relay output); Iridium always. Both send
    # the boat status, and both replies carry shore's commands to the Controller's inbox.
    ctl = None
    report = lambda: ctl.report() if ctl else {}
    on_reply = lambda reply: ctl and ctl.on_reply(reply)
    starlink = StarlinkLink(a.starlink_url, is_on=lambda: bridge.snapshot().starlink_on,
                            report=report, on_reply=on_reply)
    iridium = IridiumLink(a.iridium_url, report=report, on_reply=on_reply)
    ctl = Controller(bridge, starlink, no_plan=a.no_plan)
    starlink.start()
    iridium.start()

    run(bridge, starlink, iridium, ctl, a.tick)


if __name__ == "__main__":
    main()
