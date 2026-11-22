#!/usr/bin/env python3
"""Forecast Scout's battery voltage from the mission plan.

    python forecast/forecast.py --from 2025-07-23T00:00Z --hours 24

Prints the start, final and lowest voltage, and saves a plot to forecast/output/.
"""
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from model import DATA_DIR, Model  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent / "output"


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Forecast Scout's battery voltage from the mission plan.")
    p.add_argument("--from", dest="start", required=True, help="start time, UTC, e.g. 2025-07-23T00:00Z")
    p.add_argument("--hours", type=float, required=True, help="how many hours to forecast")
    p.add_argument("--data", default=DATA_DIR, help=f"data folder (default: {DATA_DIR})")
    p.add_argument("--out", default=OUT_DIR, help=f"folder for the plot (default: {OUT_DIR})")
    return p.parse_args(argv)


def plot(f, measured, start, out_path):
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    ax.plot(measured.index, measured.values, ".", ms=2, color="k", label="measured")
    ax.plot(f.index, f.V, color="#d62728", lw=1.5, label="forecast")
    lo = f.V.idxmin()
    ax.plot(lo, f.V[lo], "v", color="#d62728", ms=9)
    ax.annotate(f"lowest {f.V[lo]:.2f} V", (lo, f.V[lo]), xytext=(8, 6), textcoords="offset points", color="#d62728")
    ax.set_ylabel("battery voltage (V)")
    ax.set_title(f"Forecast from {start:%Y-%m-%d %H:%M} UTC, {(f.index[-1] - f.index[0]).total_seconds() / 3600:.0f} h")
    ax.legend(loc="best")
    ax2.plot(f.index, f.P_solar, color="#f2b701", lw=1.2, label="solar (forecast)")
    ax2.plot(f.index, f.P_load, color="#1f77b4", lw=1.2, label="load (from the plan)")
    ax2.set_ylabel("power (W)")
    ax2.legend(loc="upper right")
    for a in (ax, ax2):
        a.grid(alpha=0.3)
    ax2.xaxis.set_major_formatter(mdates.DateFormatter("%d %b\n%H:%M"))
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def main(argv=None):
    args = parse_args(argv)
    start = pd.Timestamp(args.start)
    start = start.tz_localize("UTC") if start.tz is None else start.tz_convert("UTC")
    if args.hours <= 0:
        sys.exit("--hours must be positive")

    model = Model(args.data)
    t_first, t_last = model.volts.index[0], model.volts.index[-1]
    if not (t_first - pd.Timedelta("1h") <= start <= t_last):
        sys.exit(f"--from must be inside the data: {t_first:%Y-%m-%d %H:%M} to {t_last:%Y-%m-%d %H:%M} UTC")
    plan_end = model.mission["to"].max()

    v0, how = model.start_voltage(start)
    f = model.run(start, args.hours, v0=v0)
    end = f.index[-1]
    lo = f.V.idxmin()
    measured = model.volts[f.index[0]: end]

    OUT_DIR_ = Path(args.out)
    OUT_DIR_.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR_ / f"forecast_{start:%Y%m%dT%H%M}Z_{args.hours:g}h.png"
    plot(f, measured, start, out_path)

    print(f"Forecast from {start:%Y-%m-%d %H:%M} UTC for {args.hours:g} h (to {end:%Y-%m-%d %H:%M})")
    print(f"  start voltage:  {v0:.2f} V ({how})")
    print(f"  final voltage:  {f.V.iloc[-1]:.2f} V")
    print(f"  lowest voltage: {f.V[lo]:.2f} V at {lo:%Y-%m-%d %H:%M}")
    print(f"  energy: solar in {(f.P_solar.sum() / 60) * 1e-3:.2f} kWh, load out {(f.P_load.sum() / 60) * 1e-3:.2f} kWh")
    if end > plan_end:
        print(f"  note: the plan ends {plan_end:%Y-%m-%d %H:%M}; after that the motor and Starlink are taken as off")
    if len(measured):
        near_end = model.volts[end - pd.Timedelta("30min"): end]
        if len(near_end):
            print(f"  measured at the end: {near_end.median():.2f} V (forecast error {near_end.median() - f.V.iloc[-1]:+.2f} V)")
    print(f"  plot: {out_path}")


if __name__ == "__main__":
    main()
