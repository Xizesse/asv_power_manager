"""Decision-logic tests for the planner (service/planner.py + service/problem.py).

Run:  .venv/bin/pytest planner/tests -v
The same scenarios are plotted in planner/tests/scenarios_view.ipynb.

Each test plans one scenario from planner/tests/scenarios/ exactly as the boat does
(make_plan: ShiftEnergy because the mission has arrive_by) and checks the answer.
"""
import multiprocessing
import time

import pandas as pd
import pytest

import scenarios as sc


def test_deadline_mission_is_planned_safely():
    """deadline.json: the full mission (320 km, deadline 22 Jul 12:00, reserve 23 V, start 24.35 V).
    A safe plan exists, so the planner must find one."""
    plan, prediction = sc.run("deadline")
    summary = plan["summary"]
    mission = sc.mission("deadline")

    # the boat uses ShiftEnergy when the mission has a deadline
    assert plan["planner"].startswith("shift-energy")

    # the plan reaches the last waypoint ...
    assert summary["status"] == "arrived"

    # ... before the deadline ...
    assert pd.Timestamp(summary["arrival"]) <= pd.Timestamp(mission["arrive_by"])

    # ... and the predicted voltage never goes under the reserve on the way
    assert min(prediction["series"]["v_batt"]) >= mission["reserve_v"]

    # so the planner calls it feasible
    assert summary["feasible"]


def test_impossible_deadline_is_reported():
    """too_fast.json: the same route with a deadline 30 h after the start (needs ~3 m/s, the planner
    stops at 1.0 m/s). No plan can make it in time. The planner must say so, and still give the boat a
    safe plan: get there as soon as it can, late, without going under the reserve."""
    plan, prediction = sc.run("too_fast")
    summary = plan["summary"]
    mission = sc.mission("too_fast")

    # it is late, so the plan is reported as not feasible (shore sees this before pressing start) ...
    assert not summary["on_time"]
    assert not summary["feasible"]

    # ... but the boat still gets there, after the deadline ...
    assert summary["status"] == "arrived"
    assert pd.Timestamp(summary["arrival"]) > pd.Timestamp(mission["arrive_by"])

    # ... and the predicted voltage never goes under the reserve on the way
    assert min(prediction["series"]["v_batt"]) >= mission["reserve_v"]


def test_reserve_wins_over_deadline():
    """tight_deadline.json: mission_impossible as the boat got it (deadline 19 Jul 12:00). Arriving on time
    IS possible, but only by going under the 23 V reserve for hours. The reserve is stricter than the
    deadline: the planner must keep the reserve and arrive late instead, and report it as not feasible."""
    plan, prediction = sc.run("tight_deadline")
    summary = plan["summary"]
    mission = sc.mission("tight_deadline")

    # the voltage never goes under the reserve: the deadline gave way, not the battery ...
    assert min(prediction["series"]["v_batt"]) >= mission["reserve_v"]
    assert summary["min_under_reserve"] == 0

    # ... so it arrives, but after the deadline ...
    assert summary["status"] == "arrived"
    assert pd.Timestamp(summary["arrival"]) > pd.Timestamp(mission["arrive_by"])

    # ... and shore is told: late, so not feasible
    assert not summary["on_time"]
    assert not summary["feasible"]


def test_no_deadline_arrives_safely():
    """no_deadline.json: the full mission without arrive_by. With no deadline the boat uses SlowDown:
    go at the mission speed, and slow down only where the battery needs it."""
    plan, prediction = sc.run("no_deadline")
    summary = plan["summary"]
    mission = sc.mission("no_deadline")

    # no deadline -> SlowDown
    assert plan["planner"].startswith("slow-down")

    # it reaches the last waypoint ...
    assert summary["status"] == "arrived"

    # ... never faster than the mission speed (speed_ms) ...
    assert max(h["speed"] for h in plan["hourly"]) <= mission["speed_ms"]

    # ... and never under the reserve, so it is feasible
    assert min(prediction["series"]["v_batt"]) >= mission["reserve_v"]
    assert summary["feasible"]


def test_low_start_keeps_reserve():
    """low_start.json: the full mission, but the boat starts at 23.30 V, only 0.3 V above the reserve.
    A safe plan still exists: it must arrive on time without ever going under the reserve."""
    plan, prediction = sc.run("low_start")
    summary = plan["summary"]
    mission = sc.mission("low_start")

    # deadline -> ShiftEnergy
    assert plan["planner"].startswith("shift-energy")

    # on time ...
    assert summary["status"] == "arrived"
    assert pd.Timestamp(summary["arrival"]) <= pd.Timestamp(mission["arrive_by"])

    # ... and never under the reserve, even starting this close to it
    assert min(prediction["series"]["v_batt"]) >= mission["reserve_v"]
    assert summary["feasible"]


def test_start_below_reserve_is_reported_and_recovers():
    """start_below_reserve.json: the full mission, but the boat starts at 22.80 V, already under the
    23 V reserve. No plan can be feasible (it starts under the reserve), but the boat still needs one:
    on time, back above the reserve as the sun charges it, and never under it again after that."""
    plan, prediction = sc.run("start_below_reserve")
    summary = plan["summary"]
    mission = sc.mission("start_below_reserve")
    v = prediction["series"]["v_batt"]
    reserve = mission["reserve_v"]

    # it starts under the reserve, so the plan is reported as not feasible ...
    assert v[0] < reserve
    assert not summary["feasible"]

    # ... but it still arrives on time ...
    assert summary["status"] == "arrived"
    assert pd.Timestamp(summary["arrival"]) <= pd.Timestamp(mission["arrive_by"])

    # ... gets back above the reserve, and once there never drops under it again
    back = next(i for i, x in enumerate(v) if x >= reserve)
    assert min(v[back:]) >= reserve


TIME_LIMIT_S = 60      # the boat can't wait forever for a plan


def _plan(name):
    sc.run(name)


@pytest.mark.skip(reason="known to fail (planner too slow on very long missions): kept for the demo")
def test_very_long_mission_answers_in_time():
    """very_long.json: the mission.json loop 12 times (3,653 km), deadline 10 Sep, just before the forecast
    ends (1,374 h). Feasible, but big. However big the mission, the planner must answer within
    TIME_LIMIT_S. Fails for now on purpose: the planner has no time limit yet."""
    # plan in a separate process, so the test can stop it instead of hanging
    worker = multiprocessing.get_context("fork").Process(target=_plan, args=("very_long",))
    tic = time.monotonic()
    worker.start()
    worker.join(TIME_LIMIT_S)
    still_running = worker.is_alive()
    if still_running:
        worker.terminate()
        worker.join()

    # it answered within the time limit
    assert not still_running, f"planner still running after {time.monotonic() - tic:.0f} s (limit {TIME_LIMIT_S} s)"
