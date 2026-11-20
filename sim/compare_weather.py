"""Plot ERA5 (truth) vs the forecast issued N days earlier, for a short window.

Usage:
    .venv/bin/python scripts/compare_weather.py
    .venv/bin/python scripts/compare_weather.py --start 2025-07-15 --days 2 --lead 1
"""
import argparse
import json
import urllib.parse
import urllib.request
from datetime import date, timedelta

import matplotlib.pyplot as plt
import pandas as pd

ARCHIVE_URL = "https://archive-api.open-meteo.com/v1/archive"            # ERA5 = truth
PREVIOUS_RUNS_URL = "https://previous-runs-api.open-meteo.com/v1/forecast"  # past forecasts

VARS = ["shortwave_radiation", "cloud_cover", "wind_speed_10m", "wind_direction_10m"]
LABELS = {
    "shortwave_radiation": "Solar irradiance (W/m²)",
    "cloud_cover": "Cloud cover (%)",
    "wind_speed_10m": "Wind speed 10 m (m/s)",
    "wind_direction_10m": "Wind direction (° from)",
}


def get(url, params):
    full = url + "?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(full, timeout=30) as r:
        data = json.load(r)
    if data.get("error"):
        raise RuntimeError(data.get("reason"))
    df = pd.DataFrame(data["hourly"])
    df["time"] = pd.to_datetime(df["time"], utc=True)
    return df.set_index("time")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat", type=float, default=60.5)
    ap.add_argument("--lon", type=float, default=1.5)
    ap.add_argument("--start", default="2025-07-15")
    ap.add_argument("--days", type=int, default=2)
    ap.add_argument("--lead", type=int, default=1, help="forecast issued this many days earlier (1-7)")
    a = ap.parse_args()

    start = date.fromisoformat(a.start)
    end = start + timedelta(days=a.days - 1)
    common = {
        "latitude": a.lat, "longitude": a.lon,
        "start_date": start.isoformat(), "end_date": end.isoformat(),
        "timezone": "UTC", "wind_speed_unit": "ms",
    }

    truth = get(ARCHIVE_URL, {**common, "hourly": ",".join(VARS), "models": "era5"})

    fc_vars = [f"{v}_previous_day{a.lead}" for v in VARS]
    fc = get(PREVIOUS_RUNS_URL, {**common, "hourly": ",".join(fc_vars)})
    fc.columns = [c.replace(f"_previous_day{a.lead}", "") for c in fc.columns]

    fig, axes = plt.subplots(len(VARS), 1, figsize=(11, 9), sharex=True)
    for ax, v in zip(axes, VARS):
        style = "." if v == "wind_direction_10m" else "-"
        ax.plot(truth.index, truth[v], style, color="black", label="ERA5 (truth)")
        ax.plot(fc.index, fc[v], style, color="tab:orange", label=f"Forecast, {a.lead} day ahead")
        ax.set_ylabel(LABELS[v])
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{a.lat}°N {a.lon}°E, {start} → {end} (UTC)")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
