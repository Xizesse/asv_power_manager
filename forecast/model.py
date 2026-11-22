"""Scout energy model: battery voltage forecast from the mission plan.

Copied from notebooks/12_prediction_model.ipynb (the notebooks stay the reference).
Every number below comes from a notebook; if a notebook is refit, update it here by hand.

Two ways in:
    Model().run(start, hours)   inputs from the mission files (forecast.py)
    forecast(grid, p_solar, starlink, v, k, v0=... | ocv0=...)   any inputs (the planner)

    P_load  = P_hotel + P_starlink * Starlink + k_mode * v_line**2
    P_solar = peak_day * cos(pi * (t - t_noon) / L)**p   (0 at night)
    P_net   = eta * P_solar - P_load - P_drain
    OCV    += P_net * dt / k,   V = OCV + R * P_net / V,   no charging above v_max
"""
from pathlib import Path

import numpy as np
import pandas as pd

# ---- parameters ----
HOTEL_W = 10.0                                   # nb 06: MANUAL, Starlink off
STARLINK_W = 22.0                                # nb 06: MANUAL, Starlink on minus off
K_MODE = {"AUTO": 62.7, "mode 17": 60.6}         # nb 08: k·v² on the mission line speed, W per (m/s)²; MANUAL = 0
BATT = dict(k=420.3, eta=1.138, p_drain=-0.312, R=0.03921, v_max=25.0)   # nb 11, model B1
START_WINDOW = pd.Timedelta("30min")             # starting voltage: median of the readings in the 30 min before the start

DATA_DIR = Path(__file__).resolve().parent.parent / "data"   # telemetry here, mission files in data/mission/


class Model:
    """Loads the plan (mission, Starlink sessions, solar forecast) and the telemetry, and runs forecasts."""

    def __init__(self, data_dir=DATA_DIR):
        data_dir = Path(data_dir)

        # mission (nb 04): one line per mode/arming change or speed step; motor on in AUTO and mode 17
        m = pd.read_csv(data_dir / "mission" / "mission_modes.csv", parse_dates=["from", "to"])
        m["k"] = m["mode"].map(K_MODE).fillna(0.0)
        m["v"] = np.where(m.k > 0, m["SOG (m/s)"].fillna(0.0), 0.0)
        self.mission = m

        # Starlink plan (nb 05): every session as an on-window
        self.starlink = pd.read_csv(data_dir / "mission" / "mission_starlink.csv", parse_dates=["from", "to"])

        # solar forecast (nb 10): shared shape, one peak per day; the mean peak on days without one
        path = data_dir / "mission" / "solar_daily_peaks.csv"
        with open(path) as f:
            hdr = dict(kv.split("=") for kv in f.readline().split(";")[-1].split())
        self.t_noon, self.L, self.p, self.peak_mean = (float(hdr[k]) for k in ["t_noon", "L", "p", "mean_peak_W"])
        peaks = pd.read_csv(path, comment="#", index_col="day").peak_W
        peaks.index = pd.to_datetime(peaks.index).tz_localize("UTC")
        self.peaks = peaks

        # telemetry: only the battery voltage is used (starting point, and the measured line on the plot)
        df = pd.read_csv(data_dir / "scout_telemetry_2025-07.csv", na_values="null",
                         usecols=["created_at", "voltage_battery1_v"])
        df["t"] = pd.to_datetime(df["created_at"], format="ISO8601", utc=True)
        self.volts = df.dropna(subset=["voltage_battery1_v"]).set_index("t").voltage_battery1_v.sort_index()

    # ---- inputs, per minute ----
    def motor(self, grid):
        """Mission line speed (m/s) and motor factor k (W per (m/s)², 0 = motor off) at each minute."""
        m = self.mission
        idx = np.searchsorted(m["from"].values, grid.values, side="right") - 1
        ok = (idx >= 0) & (grid.values < m["to"].values[idx.clip(0)])
        v = np.where(ok, m.v.values[idx.clip(0)], 0.0)
        k = np.where(ok, m.k.values[idx.clip(0)], 0.0)
        return v, k

    def starlink_share(self, grid):
        """Share of each minute [t, t + 1 min) inside a planned Starlink session (0-1)."""
        sl = np.zeros(len(grid))
        for r in self.starlink.itertuples():
            sl += np.clip((np.minimum(grid + pd.Timedelta("1min"), r.to) - np.maximum(grid, r._1)).total_seconds() / 60, 0, 1)
        return sl

    def solar(self, grid):
        """Solar power (W): the day's peak times the shared shape."""
        hour = (grid - grid.floor("D")).total_seconds() / 3600
        x = np.pi * (np.asarray(hour, dtype=float) - self.t_noon) / self.L
        shape = np.where(np.abs(x) < np.pi / 2, np.cos(np.clip(x, -np.pi / 2, np.pi / 2)) ** self.p, 0.0)
        return self.peaks.reindex(grid.floor("D")).fillna(self.peak_mean).values * shape

    def start_voltage(self, t):
        """Measured battery voltage at t: median of the readings in the 30 min before t (no look-ahead).
        If there are none (start of the data), the first reading after t."""
        before = self.volts[t - START_WINDOW: t]
        if len(before):
            return float(before.median()), "median of the readings in the 30 min before"
        after = self.volts[t:]
        if not len(after):
            raise ValueError(f"no battery voltage reading at or after {t}")
        return float(after.iloc[0]), f"first reading after (at {after.index[0]:%Y-%m-%d %H:%M})"

    # ---- the forecast ----
    def run(self, start, hours, v0=None):
        """Minute-by-minute forecast from `start` for `hours`, with the mission plan as input.
        Returns a DataFrame indexed by time (see forecast())."""
        start = pd.Timestamp(start)
        start = start.tz_localize("UTC") if start.tz is None else start.tz_convert("UTC")
        grid = pd.date_range(start.floor("min"), start + pd.Timedelta(hours=hours), freq="1min")
        if v0 is None:
            v0, _ = self.start_voltage(start)
        v, k = self.motor(grid)
        return forecast(grid, self.solar(grid), self.starlink_share(grid), v, k, v0=v0)


# ---- the run itself: any inputs, per minute ----
def load_w(starlink, v, k):
    """Load (W): hotel + Starlink share * Starlink + k * v**2 (k = 0 when the motor is off)."""
    return HOTEL_W + STARLINK_W * np.asarray(starlink, dtype=float) + np.asarray(k, dtype=float) * np.asarray(v, dtype=float) ** 2


def forecast(grid, p_solar, starlink, v, k, v0=None, ocv0=None):
    """Battery voltage minute by minute for the given inputs (arrays the length of `grid`, or scalars).

    grid:     DatetimeIndex, 1-minute steps
    p_solar:  solar power (W)
    starlink: Starlink share of each minute (0-1)
    v:        speed (m/s); k: motor factor W per (m/s)², e.g. K_MODE["AUTO"], 0 = motor off
    start:    v0 = measured terminal voltage (OCV seeded from the first minute's current), or
              ocv0 = open-circuit voltage to continue from (e.g. the OCV column of an earlier run)

    Returns a DataFrame: starlink, v, P_motor, P_load, P_solar, V (terminal), OCV (after the minute)."""
    n = len(grid)
    f = pd.DataFrame(index=grid)
    f["starlink"] = np.broadcast_to(np.asarray(starlink, dtype=float), n)
    f["v"] = np.broadcast_to(np.asarray(v, dtype=float), n)
    f["P_motor"] = np.broadcast_to(np.asarray(k, dtype=float), n) * f.v ** 2
    f["P_load"] = HOTEL_W + STARLINK_W * f.starlink + f.P_motor
    f["P_solar"] = np.broadcast_to(np.asarray(p_solar, dtype=float), n)
    if (v0 is None) == (ocv0 is None):
        raise ValueError("give exactly one of v0 (measured voltage) or ocv0 (OCV to continue from)")

    k_wh, eta, p_drain, R, vmax = BATT["k"], BATT["eta"], BATT["p_drain"], BATT["R"], BATT["v_max"]
    V, OCV, ocv = np.empty(n), np.empty(n), ocv0
    for i, (pl, ps) in enumerate(zip(f.P_load.values, f.P_solar.values)):
        p_net = eta * ps - pl - p_drain
        if ocv is None:
            ocv = v0 - R * p_net / v0
        v_t = ocv + R * p_net / max(ocv, 1)
        if p_net > 0 and v_t > vmax:                     # charge cap: extra solar is lost
            p_net = max((vmax - ocv) / R, 0) * vmax
            v_t = max(min(v_t, vmax), ocv)
        V[i] = v_t
        ocv += p_net / 60 / k_wh
        OCV[i] = ocv
    f["V"], f["OCV"] = V, OCV
    return f
