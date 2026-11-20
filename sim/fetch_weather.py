"""Download the weather for the simulation once and save it to CSV.

  data/weather/truth.csv     ERA5 reanalysis: what actually happened (env sim only)
  data/weather/forecast.csv  forecasts issued 1, 2, 3 days earlier (boat side only)

Usage (from the repo root):
    .venv/bin/python sim/fetch_weather.py
    .venv/bin/python sim/fetch_weather.py --start 2025-07-14 --days 17 --lat 60.5 --lon 1.5
"""
import argparse
import json
import urllib.parse
import urllib.request
from datetime import date, timedelta
from pathlib import Path

import pandas as pd

ARCHIVE_URL = "https://archive-api.open-meteo.com/v1/archive"
PREVIOUS_RUNS_URL = "https://previous-runs-api.open-meteo.com/v1/forecast"
VARS = ["shortwave_radiation", "cloud_cover", "wind_speed_10m", "wind_direction_10m"]
LEADS = [1, 2, 3]
OUT_DIR = Path("data/weather")


def get(url, params):
    full = url + "?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(full, timeout=60) as r:
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
    ap.add_argument("--start", default="2025-07-14")
    ap.add_argument("--days", type=int, default=17)
    a = ap.parse_args()

    start = date.fromisoformat(a.start)
    end = start + timedelta(days=a.days - 1)
    common = {
        "latitude": a.lat, "longitude": a.lon,
        "start_date": start.isoformat(), "end_date": end.isoformat(),
        "timezone": "UTC", "wind_speed_unit": "ms",
    }

    truth = get(ARCHIVE_URL, {**common, "hourly": ",".join(VARS), "models": "era5"})

    fc_vars = [f"{v}_previous_day{n}" for n in LEADS for v in VARS]
    fc = get(PREVIOUS_RUNS_URL, {**common, "hourly": ",".join(fc_vars)})
    fc.columns = [c.replace("_previous_day", "_d") for c in fc.columns]  # e.g. cloud_cover_d1

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    meta = f"# lat={a.lat} lon={a.lon} start={start} days={a.days} source=open-meteo\n"
    for name, df in [("truth.csv", truth), ("forecast.csv", fc)]:
        path = OUT_DIR / name
        with open(path, "w") as f:
            f.write(meta)
            df.to_csv(f, index_label="time")
        missing = int(df.isna().sum().sum())
        print(f"{path}: {len(df)} rows, {len(df.columns)} columns, {missing} missing values")


if __name__ == "__main__":
    main()
