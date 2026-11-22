"""The planning problem: everything fixed for one planning run, and running a plan.
Used by service/planner.py (the planners, make_plan for the boat) and planner/planner.ipynb.

Problem(mission, forecast) takes:
  mission   dict. From shore (operator inputs):
              waypoints    list of {lat, lon}; the first one is the start  -> route_km, leg_end_km
              reserve_v    lowest voltage allowed (V)                       -> reserve
              arrive_by    optional deadline at the last waypoint (UTC)     -> deadline, n_hours
              speed_ms     default speed; NOT used by Problem
            Added by the boat when it plans:
              start_time   its autopilot time (UTC)                          -> t0, minute grid t
              start_v      its measured battery voltage (V)                  -> start_v
              starlink     its Starlink schedule {every_h, first_h, at_min, on_min} -> starlink (0/1 per minute)
  forecast  weather DataFrame from read_forecast(): shortwave_radiation_d1/_d2/_d3 (W/m²), hourly
            -> p_solar = K_SOLAR x irradiance, W per minute (day n of the mission uses _d(n+1), max _d3);
               its last hour sets horizon_h
            -> Should include wind and currents, not doing it for now


"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "forecast"))
import model as fm  # noqa: E402  forecast/model.py: parameters and the run itself

K_SOLAR = 0.164                     # m², irradiance -> panel W (nb 10 section 4)
K_AUTO = fm.K_MODE["AUTO"]          # the mission runs in AUTO; v = 0 -> motor off


# ---- inputs ----
def haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp, dl = p2 - p1, np.radians(lon2 - lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * R * np.arcsin(np.sqrt(a))


def read_forecast(path):
    """Weather file (data/weather/*.csv): hourly means over the hour before the timestamp -> middle of the hour."""
    w = pd.read_csv(path, comment="#", parse_dates=["time"]).set_index("time")
    w.index = w.index - pd.Timedelta(minutes=30)
    return w


def starlink_on(t, sched):
    """Starlink on the schedule at time t (a datetime or a DatetimeIndex)."""
    minutes_since = ((t.hour - sched["first_h"]) % sched["every_h"]) * 60 + t.minute - sched["at_min"]
    return (minutes_since >= 0) & (minutes_since < sched["on_min"])


# ---- the problem ----
class Problem:
    """Everything fixed for one planning run (inputs: see the top of this file).
    run() turns hourly speeds into per-minute inputs and hands them to fm.forecast().

    State = {v0, ocv, km}: the first run starts from the measured voltage (v0); a run that continues
    an earlier one starts from its OCV (ocv)."""

    def __init__(self, mission, forecast):
        self.mission = mission
        self.t0 = pd.Timestamp(mission["start_time"])
        self.start_v = mission["start_v"]
        self.reserve = mission["reserve_v"]
        w = pd.DataFrame(mission["waypoints"])
        self.leg_end_km = np.cumsum(haversine_km(w.lat.values[:-1], w.lon.values[:-1],
                                                 w.lat.values[1:], w.lon.values[1:]))
        self.route_km = float(self.leg_end_km[-1])

        # minute grid: from the start to the end of the forecast
        end = forecast.index[-1]
        if self.t0 < forecast.index[0] or self.t0 >= end:
            raise ValueError(f"start {self.t0} is outside the forecast ({forecast.index[0]} -> {end})")
        self.horizon_h = int((end - self.t0) / pd.Timedelta(hours=1))
        self.t = pd.date_range(self.t0, periods=self.horizon_h * 60, freq="min")

        # optional deadline: plans are n_hours long
        self.deadline = pd.Timestamp(mission["arrive_by"]) if mission.get("arrive_by") else None
        if self.deadline is not None:
            self.n_hours = int((self.deadline - self.t0) / pd.Timedelta(hours=1))
            if self.n_hours <= 0:
                raise ValueError(f"arrive_by {self.deadline} is not after the start {self.t0}")
            if self.n_hours > self.horizon_h:
                raise ValueError(f"arrive_by {self.deadline} is after the end of the forecast ({end})")
        else:
            self.n_hours = self.horizon_h

        # plan solar: day n uses the forecast made n+1 days ahead (max 3)
        lead = np.minimum((self.t.normalize() - self.t0.normalize()).days + 1, 3)

        def at_minutes(s):
            return s.reindex(s.index.union(self.t)).interpolate(method="time").reindex(self.t).values

        G = np.choose(lead - 1, [at_minutes(forecast[f"shortwave_radiation_d{d}"]) for d in (1, 2, 3)])
        self.p_solar = K_SOLAR * np.clip(G, 0, None)
        self.starlink = starlink_on(self.t, mission["starlink"]).astype(float)

    def start_state(self):
        return {"v0": self.start_v, "ocv": None, "km": 0.0}

    def run(self, speeds, state, h0=0, check_reserve=False):
        """Hours h0, h0+1, ... at the given hourly speeds, run by fm.forecast().
        Returns (state after, minute table, status).
        status: 'arrived' | 'below_reserve' (only if check_reserve) | 'out_of_data' | 'ok' (plan ran out first)"""
        i0 = h0 * 60
        n = min(len(speeds) * 60, len(self.t) - i0)
        if n <= 0:
            return state, None, "out_of_data"
        sl = slice(i0, i0 + n)
        v = np.repeat(np.asarray(speeds, dtype=float), 60)[:n]
        start = {"v0": state["v0"]} if state["ocv"] is None else {"ocv0": state["ocv"]}
        df = fm.forecast(self.t[sl], self.p_solar[sl], self.starlink[sl], v, K_AUTO * (v > 0), **start)
        df["km"] = state["km"] + np.cumsum(v) * 60 / 1000

        status = "ok" if n == len(speeds) * 60 else "out_of_data"
        arrive = np.flatnonzero(df.km.values >= self.route_km)
        below = np.flatnonzero(df.V.values < self.reserve) if check_reserve else []
        if len(arrive) and (not len(below) or arrive[0] <= below[0]):
            df, status = df.iloc[:arrive[0] + 1], "arrived"
        elif len(below):
            df, status = df.iloc[:below[0] + 1], "below_reserve"
        last = df.iloc[-1]
        return {"v0": None, "ocv": float(last.OCV), "km": float(last.km)}, df, status

    def evaluate(self, plan):
        """Whole plan from the start -> (minute table, summary)."""
        _, df, status = self.run(plan, self.start_state())
        arrived = status == "arrived"
        summary = {
            "status": status,
            "arrival": df.index[-1] if arrived else None,
            "hours": len(df) / 60 if arrived else None,
            "min_V": float(df.V.min()),
            "end_V": float(df.V.iloc[-1]),
            "min_under_reserve": int((df.V < self.reserve).sum()),
            "on_time": bool(arrived and (self.deadline is None or df.index[-1] <= self.deadline)),
        }
        summary["feasible"] = bool(summary["on_time"] and df.V.min() >= self.reserve)
        return df, summary
