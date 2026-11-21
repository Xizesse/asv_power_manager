"""Planner, extracted from planner/planner.ipynb (Part 2). Pure functions, no plotting.

A plan = one speed per hour from SPEEDS. The boat-side models are the same as the notebook:
solar = 0.164 m^2 x forecast irradiance, load = 10 W + 22 W Starlink + k*v, battery B1.

The boat only ever sees the *forecast* (never ERA5).
TEMPORARY: the forecast is read from data/weather/forecast.csv on disk. On the real boat it has to
arrive from shore (forecast pack, design log section 5).

Main entry point: make_plan(mission, forecast, now_unix, batt_v, lat, lon) -> JSON-able dict.
"""
import math
from datetime import datetime, timezone

import numpy as np
import pandas as pd

# ---- models (notebook sections 4-7; nb 03/04/05) ----
K_SOLAR = 0.164                     # m^2
HOTEL_W = 10.0
STARLINK_W = 22.0
LOAD_MODELS = {"k·v": (1, 64.81), "k·v²": (2, 54.41), "k·v³": (3, 39.32)}
LOAD_MODEL = "k·v"
BATT_ETA = 1.138
BATT_P_DRAIN_W = -0.312
BATT_K_WH_PER_V = 420.3
R_INTERNAL_OHM = 0.03921
V_CAP = 25.0
V_EMPTY = 20.0

SPEEDS = np.round(np.arange(0, 1.5 + 1e-9, 0.2), 1)   # 0.0 ... 1.4 m/s
SERIES_EVERY_MIN = 1                                    # resolution of the predicted curve sent to shore
                                                        # (same 1-min bins as the telemetry, service/minute_bins.py)


def haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp, dl = p2 - p1, np.radians(lon2 - lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * R * np.arcsin(np.sqrt(a))


def read_forecast(path):
    w = pd.read_csv(path, comment="#", parse_dates=["time"]).set_index("time")
    w.index = w.index - pd.Timedelta(minutes=30)        # hour-mean -> middle of the hour
    return w


def starlink_on(t, sched):
    minutes_since = ((t.hour - sched["first_h"]) % sched["every_h"]) * 60 + t.minute - sched["at_min"]
    return (minutes_since >= 0) & (minutes_since < sched["on_min"])


class Problem:
    """Everything fixed for one planning run; simulates a plan minute by minute."""

    def __init__(self, mission, forecast, load_model=LOAD_MODEL):
        self.mission = mission
        self.t0 = pd.Timestamp(mission["start_time"])
        self.start_v = mission["start_v"]
        self.reserve = mission["reserve_v"]
        w = pd.DataFrame(mission["waypoints"])
        self.leg_end_km = np.cumsum(haversine_km(w.lat.values[:-1], w.lon.values[:-1],
                                                 w.lat.values[1:], w.lon.values[1:]))
        self.route_km = float(self.leg_end_km[-1])

        end = forecast.index[-1]
        if self.t0 < forecast.index[0] or self.t0 >= end:
            raise ValueError(f"start {self.t0} is outside the forecast ({forecast.index[0]} -> {end})")
        self.horizon_h = int((end - self.t0) / pd.Timedelta(hours=1))
        self.t = pd.date_range(self.t0, periods=self.horizon_h * 60, freq="min")

        self.deadline = pd.Timestamp(mission["arrive_by"]) if mission.get("arrive_by") else None
        if self.deadline is not None:
            self.n_hours = int((self.deadline - self.t0) / pd.Timedelta(hours=1))
            if self.n_hours <= 0:
                raise ValueError(f"arrive_by {self.deadline} is not after the start {self.t0}")
            if self.n_hours > self.horizon_h:
                raise ValueError(f"arrive_by {self.deadline} is after the end of the forecast ({end})")
        else:
            self.n_hours = self.horizon_h

        lead = np.minimum((self.t.normalize() - self.t0.normalize()).days + 1, 3)

        def at_minutes(s):
            return s.reindex(s.index.union(self.t)).interpolate(method="time").reindex(self.t).values

        G = np.choose(lead - 1, [at_minutes(forecast[f"shortwave_radiation_d{d}"]) for d in (1, 2, 3)])
        self.p_solar = K_SOLAR * np.clip(G, 0, None)
        self.starlink = starlink_on(self.t, mission["starlink"])
        self.p_fixed = HOTEL_W + STARLINK_W * self.starlink
        self.load_model = load_model
        self.n, self.k = LOAD_MODELS[load_model]

    def start_state(self):
        return {"ocv": self.start_v, "v_term": self.start_v, "km": 0.0}

    def motor_w(self, v):
        return self.k * v ** self.n if v > 0 else 0.0

    def run(self, speeds, state, h0=0, check_reserve=False, record=False):
        """Simulate hours h0, h0+1, ... -> (state after, minute records or None, status).
        status: 'arrived' | 'below_reserve' (only if check_reserve) | 'out_of_data' | 'ok'"""
        ocv, v_term, km = state["ocv"], state["v_term"], state["km"]
        rec = [] if record else None
        for i, v in enumerate(speeds):
            p_motor = self.motor_w(v)
            for m in range(60):
                idx = (h0 + i) * 60 + m
                if idx >= len(self.t):
                    return {"ocv": ocv, "v_term": v_term, "km": km}, rec, "out_of_data"
                p_load = self.p_fixed[idx] + p_motor
                p_net = BATT_ETA * self.p_solar[idx] - p_load - BATT_P_DRAIN_W
                i_b = p_net / v_term
                v_term = ocv + R_INTERNAL_OHM * i_b
                if p_net > 0 and v_term > V_CAP:
                    i_b = max((V_CAP - ocv) / R_INTERNAL_OHM, 0.0)
                    v_term = max(ocv + R_INTERNAL_OHM * i_b, ocv)
                    p_net = i_b * v_term
                if p_net < 0 and ocv <= V_EMPTY:
                    p_net, v_term = 0.0, ocv
                ocv += p_net / 60 / BATT_K_WH_PER_V
                km += v * 60 / 1000
                if record:
                    rec.append((self.t[idx], v, self.p_solar[idx], p_load, p_net, v_term, ocv, km))
                st = {"ocv": ocv, "v_term": v_term, "km": km}
                if km >= self.route_km:
                    return st, rec, "arrived"
                if check_reserve and v_term < self.reserve:
                    return st, rec, "below_reserve"
        return {"ocv": ocv, "v_term": v_term, "km": km}, rec, "ok"

    def evaluate(self, plan):
        _, rec, status = self.run(plan, self.start_state(), record=True)
        df = pd.DataFrame(rec, columns=["t", "v", "P_solar", "P_load", "P_net", "V", "OCV", "km"]).set_index("t")
        arrived = status == "arrived"
        summary = {
            "status": status,
            "arrival": df.index[-1] if arrived else None,
            "min_V": float(df.V.min()),
            "end_V": float(df.V.iloc[-1]),
            "min_under_reserve": int((df.V < self.reserve).sum()),
            "on_time": bool(arrived and (self.deadline is None or df.index[-1] <= self.deadline)),
        }
        summary["feasible"] = bool(summary["on_time"] and df.V.min() >= self.reserve)
        return df, summary


# ---- planners (notebook section 9) ----
class ConstantSpeed:
    name = "constant"

    def solve(self, problem):
        if problem.deadline is not None:
            need = problem.route_km * 1000 / (problem.n_hours * 3600)
            ok = [v for v in SPEEDS if v >= need - 1e-9]
            return [float(ok[0] if ok else SPEEDS[-1])] * problem.n_hours
        for v in SPEEDS[::-1]:
            if v == 0:
                break
            hours = math.ceil(problem.route_km * 1000 / v / 3600)
            _, _, status = problem.run([v] * hours, problem.start_state(), check_reserve=True)
            if status == "arrived":
                return [float(v)] * hours
        return []


class SlowDown:
    def __init__(self, nominal=1.0, v_min=0.4, step=0.2):
        self.nominal, self.v_min, self.step = nominal, v_min, step
        self.name = f"slow-down ({nominal}, min {v_min})"

    def solve(self, problem):
        plan = [self.nominal] * problem.horizon_h
        while True:
            _, rec, status = problem.run(plan, problem.start_state(), check_reserve=True, record=True)
            if status != "below_reserve":
                break
            h_bad = (len(rec) - 1) // 60
            slower = [h for h in range(h_bad, -1, -1) if plan[h] > self.v_min + 1e-9]
            if not slower:
                break
            h = slower[0]
            plan[h] = round(max(plan[h] - self.step, self.v_min), 1)
        return plan[:math.ceil(len(rec) / 60)]


class ShiftEnergy:
    def __init__(self, v_min=0.4, v_max=1.0, step=0.2, max_iter=3000):
        self.v_min, self.v_max, self.step, self.max_iter = v_min, v_max, step, max_iter
        self.name = f"shift-energy ({v_min}–{v_max})"

    def _score(self, problem, plan):
        _, rec, status = problem.run(plan, problem.start_state(), record=True)
        return status == "arrived", min(r[5] for r in rec), rec

    def solve(self, problem):
        if problem.deadline is None:
            raise ValueError("ShiftEnergy needs arrive_by in the mission")
        plan = [min(max(v, self.v_min), self.v_max) for v in ConstantSpeed().solve(problem)]
        _, best, rec = self._score(problem, plan)
        for _ in range(self.max_iter):
            V = np.array([r[5] for r in rec])
            n = len(rec)
            h_worst = int(np.argmin(V)) // 60
            V_hour = [V[h * 60:(h + 1) * 60] for h in range(math.ceil(n / 60))]
            give = [h for h in range(len(plan))
                    if plan[h] < self.v_max - 1e-9
                    and (h > h_worst or (h < len(V_hour) and V_hour[h].max() >= V_CAP - 0.01))]
            mean_V = lambda h: V_hour[h].mean() if h < len(V_hour) else V[-1]
            give.sort(key=mean_V, reverse=True)
            improved = False
            for h_take in [h for h in range(min(h_worst, len(plan) - 1), -1, -1) if plan[h] > self.v_min + 1e-9]:
                trial = plan.copy()
                trial[h_take] = round(trial[h_take] - self.step, 1)
                ok, low, trial_rec = self._score(problem, trial)
                if not ok:
                    g = next((h for h in give if h != h_take), None)
                    if g is None:
                        continue
                    trial[g] = round(trial[g] + self.step, 1)
                    ok, low, trial_rec = self._score(problem, trial)
                if ok and low > best + 1e-4:
                    plan, best, rec, improved = trial, low, trial_rec, True
                    break
            if not improved:
                break
        return plan


# ---- entry point for the service ----
def _iso(ts):
    return None if ts is None else pd.Timestamp(ts).strftime("%Y-%m-%dT%H:%M:%SZ")


def make_plan(mission, forecast, now_unix, batt_v, lat, lon):
    """Plan from the boat's *current* state: start time = now (autopilot time, floored to the minute),
    start voltage = measured battery V, route = current position -> mission waypoints.
    Returns a JSON-able dict (what gets saved to disk and sent to shore)."""
    t0 = pd.Timestamp(datetime.fromtimestamp(now_unix, timezone.utc)).floor("min")
    eff = dict(mission)
    eff["start_time"] = _iso(t0)
    eff["start_v"] = round(float(batt_v), 3)
    eff["waypoints"] = [{"lat": lat, "lon": lon}] + list(mission["waypoints"])

    problem = Problem(eff, forecast)
    planner = ShiftEnergy() if problem.deadline is not None else SlowDown()
    speeds = [float(v) for v in planner.solve(problem)]
    df, s = problem.evaluate(speeds)

    # V, speed: value at the start of each bin. Power: mean over the bin (with 1-min bins = the minute itself;
    # with longer bins the mean keeps the energy right instead of hitting or missing Starlink bursts).
    bins = df.resample(f"{SERIES_EVERY_MIN}min", origin="start")
    series = bins.first()
    series[["P_solar", "P_load"]] = bins[["P_solar", "P_load"]].mean()
    t_unix = ((series.index - pd.Timestamp(0, tz="UTC")) // pd.Timedelta(seconds=1)).tolist()   # unit-safe
    return {
        "plan_id": f"plan-{t0:%Y%m%dT%H%MZ}",
        "created_t": float(now_unix),
        "planner": planner.name,
        "load_model": problem.load_model,
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
        "series": {
            "t": t_unix,
            "v_batt": [round(x, 3) for x in series.V],
            "speed": [round(x, 2) for x in series.v],
            "solar_w": [round(x, 1) for x in series.P_solar],     # 1-min means
            "load_w": [round(x, 1) for x in series.P_load],       # 1-min means
        },
    }
