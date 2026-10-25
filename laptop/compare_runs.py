#!/usr/bin/env python3
"""
sentinel-pi | laptop/compare_runs.py
Compares all motor_sweep.py runs saved in logs/ (repeatability check).

    python3 compare_runs.py              # all runs
    python3 compare_runs.py --last 3     # only the newest 3 runs

Prints mean +- standard deviation per duty and saves logs/compare_runs.png.
"""
import argparse
import csv
import statistics as st
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.robot import METRES_PER_COUNT  # noqa: E402

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

LOGDIR = Path(__file__).resolve().parent.parent / "logs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--last", type=int, default=0, help="use only the newest N runs")
    args = ap.parse_args()

    sweeps = sorted(LOGDIR.glob("sweep_*.csv"))
    if args.last:
        sweeps = sweeps[-args.last:]
    if not sweeps:
        sys.exit("No sweep_*.csv files in logs/")
    runs = []
    for f in sweeps:
        stamp = f.stem.replace("sweep_", "")
        rows = list(csv.DictReader(open(f)))
        step_f = LOGDIR / f"step_{stamp}.csv"
        step = list(csv.DictReader(open(step_f))) if step_f.exists() else []
        runs.append((stamp, rows, step))
    print(f"{len(runs)} runs: " + ", ".join(r[0] for r in runs))

    # ---- table: mean +- std per duty ----
    duties = [float(r["duty"]) for r in runs[0][1]]
    print("\n duty |   left m/s (mean +- std) |  right m/s (mean +- std)")
    for i, d in enumerate(duties):
        cols = []
        for key in ("v_left", "v_right"):
            vals = [float(r[1][i][key]) for r in runs if i < len(r[1])]
            sd = st.stdev(vals) if len(vals) > 1 else 0.0
            cols.append(f"{st.mean(vals):7.3f} +- {sd:5.3f}")
        print(f" {d:4.2f} |     {cols[0]}    |     {cols[1]}")

    # ---- plots ----
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
    for stamp, rows, step in runs:
        lab = stamp[-6:-2] if len(stamp) >= 6 else stamp   # HHMM
        d = [float(r["duty"]) for r in rows]
        ax[0].plot(d, [float(r["v_left"]) for r in rows], "o-", label=f"left {lab}")
        ax[0].plot(d, [float(r["v_right"]) for r in rows], "s--", label=f"right {lab}")
        if step:
            t = [float(x["t"]) for x in step]
            for k, side in ((1, "enc_l"), (2, "enc_r")):
                e = [int(x[side]) for x in step]
                w = 5   # 100 ms window
                ts = [t[i] for i in range(w, len(t))]
                vs = [(e[i] - e[i - w]) * METRES_PER_COUNT / (t[i] - t[i - w]) for i in range(w, len(t))]
                ax[k].plot(ts, vs, label=lab, alpha=0.8)
    ax[0].set(title="Duty sweep - all runs", xlabel="PWM duty", ylabel="speed [m/s]")
    ax[1].set(title="Step 0 -> 0.6, LEFT wheel", xlabel="time [s]", ylabel="speed [m/s]")
    ax[2].set(title="Step 0 -> 0.6, RIGHT wheel", xlabel="time [s]", ylabel="speed [m/s]")
    for a in ax:
        a.grid(True)
        a.legend(fontsize=7)
    fig.tight_layout()
    png = LOGDIR / "compare_runs.png"
    fig.savefig(png, dpi=120)
    print(f"\nPlot saved: logs/{png.name}")
    subprocess.Popen(["xdg-open", str(png)])


if __name__ == "__main__":
    main()
