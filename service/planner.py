"""Planner: the planners and the boat's entry point. The boat (control_loop.py) and planner/planner.ipynb both use it.

A plan = one speed per hour from SPEEDS. The problem (route, solar, Starlink, running a plan through
forecast/model.py) is service/problem.py; a planner only returns a list of hourly speeds.

The boat only ever sees the *forecast* (never ERA5).
TEMPORARY: the forecast is read from data/weather/forecast.csv on disk. On the real boat it has to
arrive from shore (forecast pack, design log section 5).

Main entry point for the boat: make_plan(mission, forecast, now_unix, batt_v, lat, lon) -> JSON-able dict.
"""
import math
from datetime import datetime, timezone

import numpy as np
import pandas as pd

from problem import K_AUTO, Problem, fm, read_forecast, starlink_on  # noqa: F401  (read_forecast, starlink_on: used by control_loop)

V_CAP = fm.BATT["v_max"]
SPEEDS = np.round(np.arange(0, 1.5 + 1e-9, 0.2), 1)   # 0.0 ... 1.4 m/s
SERIES_EVERY_MIN = 1                                    # resolution of the predicted curve sent to shore
                                                        # (same 1-min bins as the telemetry, service/minute_bins.py)


# ---- planners: solve(problem) -> list of hourly speeds ----
class Planner:
    name = "planner"

    def solve(self, problem):
        raise NotImplementedError


class ConstantSpeed(Planner):
    """Fastest single speed that arrives with V >= reserve. With a deadline: slowest speed that is on time."""
    name = "constant"

    def solve(self, problem):
        if problem.deadline is not None:
            need = problem.route_km * 1000 / (problem.n_hours * 3600)          # m/s to arrive on time
            ok = [v for v in SPEEDS if v >= need - 1e-9]
            return [float(ok[0] if ok else SPEEDS[-1])] * problem.n_hours
        for v in SPEEDS[::-1]:
            if v == 0:
                break
            hours = math.ceil(problem.route_km * 1000 / v / 3600)
            _, _, status = problem.run([v] * hours, problem.start_state(), check_reserve=True)
            if status == "arrived":
                return [float(v)] * hours
        return []   # no constant speed works


class Greedy(Planner):
    """Each hour: the fastest speed that survives `lookahead_h` h of drift afterwards."""

    def __init__(self, lookahead_h=24):
        self.lookahead_h = lookahead_h
        self.name = f"greedy ({lookahead_h} h drift check)"

    def solve(self, problem):
        state, plan = problem.start_state(), []
        for h in range(problem.horizon_h):
            for v in SPEEDS[::-1]:                                    # fastest first
                trial = [v] + [0.0] * self.lookahead_h                # this hour at v, then drift
                _, _, status = problem.run(trial, state, h0=h, check_reserve=True)
                if status != "below_reserve":
                    break
            plan.append(float(v))
            state, _, status = problem.run([v], state, h0=h)
            if status in ("arrived", "out_of_data"):
                break
        return plan


class SlowDown(Planner):
    """Every hour at `nominal`; slow the latest hour before the first reserve breach by one step, repeat."""

    def __init__(self, nominal=1.0, v_min=0.4, step=0.2):
        self.nominal, self.v_min, self.step = nominal, v_min, step
        self.name = f"slow-down ({nominal}, min {v_min})"

    def solve(self, problem):
        plan = [self.nominal] * problem.horizon_h
        self.iterations = 0
        while True:
            self.iterations += 1
            _, df, status = problem.run(plan, problem.start_state(), check_reserve=True)
            if status != "below_reserve":
                break                                          # feasible, or out of data / plan
            h_bad = (len(df) - 1) // 60                        # hour of the first minute under the reserve
            slower = [h for h in range(h_bad, -1, -1) if plan[h] > self.v_min + 1e-9]
            if not slower:
                break                                          # nothing left to slow: infeasible
            h = slower[0]
            plan[h] = round(max(plan[h] - self.step, self.v_min), 1)
        return plan[:math.ceil(len(df) / 60)]


class ShiftEnergy(Planner):
    """Needs arrive_by. Highest lowest voltage, on time: take speed from hours at or before the worst
    minute, give it to high-voltage hours after it (or at the cap); keep a move only if the lowest V rises."""

    def __init__(self, v_min=0.4, v_max=1.0, step=0.2, max_iter=3000):
        self.v_min, self.v_max, self.step, self.max_iter = v_min, v_max, step, max_iter
        self.name = f"shift-energy ({v_min}–{v_max})"

    def _score(self, problem, plan):
        """(on time?, lowest V, voltage per minute)"""
        _, df, status = problem.run(plan, problem.start_state())
        return status == "arrived", df.V.min(), df.V.values

    def solve(self, problem):
        if problem.deadline is None:
            raise ValueError("ShiftEnergy needs arrive_by in the mission")
        plan = [min(max(v, self.v_min), self.v_max) for v in ConstantSpeed().solve(problem)]
        _, best, V = self._score(problem, plan)
        self.iterations = 0
        while self.iterations < self.max_iter:
            self.iterations += 1
            h_worst = int(np.argmin(V)) // 60
            V_hour = [V[h * 60:(h + 1) * 60] for h in range(math.ceil(len(V) / 60))]
            # give candidates: highest mean V, after h_worst or at the cap
            give = [h for h in range(len(plan))
                    if plan[h] < self.v_max - 1e-9
                    and (h > h_worst or (h < len(V_hour) and V_hour[h].max() >= V_CAP - 0.01))]
            mean_V = lambda h: V_hour[h].mean() if h < len(V_hour) else V[-1]  # noqa: E731
            give.sort(key=mean_V, reverse=True)

            improved = False
            for h_take in [h for h in range(min(h_worst, len(plan) - 1), -1, -1) if plan[h] > self.v_min + 1e-9]:
                trial = plan.copy()
                trial[h_take] = round(trial[h_take] - self.step, 1)
                ok, low, trial_V = self._score(problem, trial)
                if not ok:                                          # late now: give the speed back elsewhere
                    g = next((h for h in give if h != h_take), None)
                    if g is None:
                        continue
                    trial[g] = round(trial[g] + self.step, 1)
                    ok, low, trial_V = self._score(problem, trial)
                if ok and low > best + 1e-4:
                    plan, best, V, improved = trial, low, trial_V, True
                    break
            if not improved:
                break
        return plan


# ---- entry point for the service ----
def _iso(ts):
    return None if ts is None or pd.isna(ts) else pd.Timestamp(ts).strftime("%Y-%m-%dT%H:%M:%SZ")


def make_plan(mission, forecast, now_unix, batt_v, lat, lon):
    """Plan from the boat's *current* state: start time = now (autopilot time, floored to the minute),
    start voltage = measured battery V, route = current position -> mission waypoints.

    Returns two JSON-able dicts from the same run (both saved on the boat and sent to shore):
      plan        the decision: plan_id, planner, mission, summary (feasible, arrival, min V, ...),
                  hourly speeds. All the boat needs to act.
      prediction  what the plan expects, per minute: battery V, speed, load (same plan_id).
                  No solar: it came from shore in the forecast, so shore works it out itself.

    Planner: ShiftEnergy with arrive_by, SlowDown without. If ShiftEnergy can't arrive on time, the
    deadline is impossible: SlowDown plans the earliest safe arrival instead, reported late (not feasible).
    """
    t0 = pd.Timestamp(datetime.fromtimestamp(now_unix, timezone.utc)).floor("min")
    eff = dict(mission)
    eff["start_time"] = _iso(t0)
    eff["start_v"] = round(float(batt_v), 3)
    eff["waypoints"] = [{"lat": lat, "lon": lon}] + list(mission["waypoints"])

    problem = Problem(eff, forecast)
    planner = ShiftEnergy() if problem.deadline is not None else SlowDown()
    speeds = [float(v) for v in planner.solve(problem)]
    df, s = problem.evaluate(speeds)

    if problem.deadline is not None and not s["on_time"]:
        # The deadline can't be met. Still give the boat a safe plan: plan again without the deadline
        # (SlowDown: earliest arrival, slowing down where needed to stay above the reserve) ...
        late = Problem({k: v for k, v in eff.items() if k != "arrive_by"}, forecast)
        planner = SlowDown()
        speeds = [float(v) for v in planner.solve(late)]
        df, s = late.evaluate(speeds)
        # ... and judge it against the real deadline: late, so not feasible
        s["on_time"] = s["arrival"] is not None and s["arrival"] <= problem.deadline
        s["feasible"] = bool(s["on_time"] and s["min_V"] >= problem.reserve)
    plan_id = f"plan-{t0:%Y%m%dT%H%MZ}"

    plan = {
        "plan_id": plan_id,
        "created_t": float(now_unix),
        "planner": planner.name,
        "load_model": f"forecast/model.py: k·v² AUTO {K_AUTO}",
        "forecast_source": "data/weather/forecast.csv (TEMPORARY: should come from shore)",
        "mission": eff,
        "route_km": round(problem.route_km, 2),
        "reserve_v": problem.reserve,
        "summary": {
            "feasible": s["feasible"],
            "status": s["status"],
            "on_time": s["on_time"],
            "arrival": _iso(s["arrival"]),
            "min_v": round(s["min_V"], 3),
            "end_v": round(s["end_V"], 3),
            "min_under_reserve": s["min_under_reserve"],
        },
        "hourly": [{"t": int(t0.timestamp()) + 3600 * h, "speed": v} for h, v in enumerate(speeds)],
    }

    # V, speed: value at the start of each bin. Load: mean over the bin (with 1-min bins = the minute itself;
    # with longer bins the mean keeps the energy right instead of hitting or missing Starlink bursts).
    bins = df.resample(f"{SERIES_EVERY_MIN}min", origin="start")
    series = bins.first()
    series["P_load"] = bins["P_load"].mean()
    t_unix = ((series.index - pd.Timestamp(0, tz="UTC")) // pd.Timedelta(seconds=1)).tolist()   # unit-safe
    prediction = {
        "plan_id": plan_id,
        "created_t": float(now_unix),
        "series": {
            "t": t_unix,
            "v_batt": [round(x, 3) for x in series.V],
            "speed": [round(x, 2) for x in series.v],
            "load_w": [round(x, 1) for x in series.P_load],       # 1-min means
        },
    }
    return plan, prediction
