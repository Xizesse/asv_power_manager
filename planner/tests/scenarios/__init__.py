"""Planner test scenarios: one JSON file per scenario in this folder.
Used by test_planner.py (pytest) and the notebook that plots them.

Each file:
  description   what the scenario is
  expect        "feasible" or "infeasible"
  start_time    the boat's autopilot time when it plans (UTC)
  start_v       its measured battery voltage (V)
  mission       the mission as shore sends it: speed_ms, reserve_v, arrive_by (optional), waypoints

The Starlink schedule is not in the files: it's the boat's setting (service/control_loop.py), added here.
run(name) plans the scenario exactly as the boat does (service/planner.py make_plan:
ShiftEnergy with arrive_by, SlowDown without), from the first waypoint.
"""
import functools
import json
import sys
from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "service"))
import planner  # noqa: E402  service/planner.py
import problem  # noqa: E402  service/problem.py
from control_loop import STARLINK_SCHEDULE  # noqa: E402  the boat's setting

FORECAST = ROOT / "data" / "weather" / "forecast.csv"

SCENARIOS = {p.stem: json.loads(p.read_text()) for p in sorted(HERE.glob("*.json"))}
FEASIBLE = [n for n, s in SCENARIOS.items() if s["expect"] == "feasible"]
INFEASIBLE = [n for n, s in SCENARIOS.items() if s["expect"] == "infeasible"]


@functools.lru_cache(maxsize=None)
def forecast():
    return problem.read_forecast(FORECAST)


def mission(name):
    """The scenario's mission plus the boat's Starlink schedule."""
    return dict(SCENARIOS[name]["mission"], starlink=STARLINK_SCHEDULE)


@functools.lru_cache(maxsize=None)
def run(name):
    """Plan the scenario as the boat does: make_plan from the first waypoint, at start_time, with start_v.
    Returns (plan, prediction), as make_plan does."""
    sc, m = SCENARIOS[name], mission(name)
    w0, rest = m["waypoints"][0], m["waypoints"][1:]
    t = pd.Timestamp(sc["start_time"]).timestamp()
    return planner.make_plan(dict(m, waypoints=rest), forecast(), t, sc["start_v"], w0["lat"], w0["lon"])


def make_problem(name):
    """The same Problem make_plan builds (for trying other plans, e.g. constant speed, against it)."""
    sc, m = SCENARIOS[name], mission(name)
    w0 = m["waypoints"][0]
    m = dict(m, start_time=sc["start_time"], start_v=sc["start_v"], waypoints=[w0] + m["waypoints"])
    return problem.Problem(m, forecast())
