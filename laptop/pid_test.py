#!/usr/bin/env python3
"""
sentinel-pi | laptop/pid_test.py   (Phase 3: speed controller test)

Runs the same speed profile twice and compares:
    OPEN LOOP   : feedforward only (motor model, kp = ki = 0)
    CLOSED LOOP : feedforward + PI controller
Needs pi/io_server.py running on the Pi. LIFT THE ROBOT (wheels in the air).

    python3 pid_test.py --host 10.45.2.233
    python3 pid_test.py --host 10.45.2.233 --kp 1.0 --ki 8     # try other gains

Speed profile: 0.25 m/s -> 0.35 m/s -> 0.20 m/s -> stop
Saves logs/pid_<time>.csv and logs/pid_<time>.png
"""
import argparse
import csv
import json
import statistics as st
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import zmq

LOGDIR = Path(__file__).resolve().parent.parent / "logs"
PROFILE = [(0.0, 0.5), (0.25, 3.0), (0.35, 3.0), (0.20, 3.0), (0.0, 1.0)]   # (m/s, seconds)


class Link:
    def __init__(self, host):
        self.ctx = zmq.Context()
        self.tel = self.ctx.socket(zmq.SUB)
        self.tel.setsockopt(zmq.SUBSCRIBE, b"")
        self.tel.connect(f"tcp://{host}:5556")
        self.cmd = self.ctx.socket(zmq.PUB)
        self.cmd.connect(f"tcp://{host}:5557")

    def run(self, msg, seconds):
        out, t_end = [], time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.cmd.send_string(json.dumps(msg))
            t_next = time.monotonic() + 0.05
            while time.monotonic() < t_next:
                if self.tel.poll(5):
                    out.append(json.loads(self.tel.recv()))
        return out

    def close(self):
        self.run({"mode": "duty", "left": 0, "right": 0}, 0.3)
        self.tel.close(0)
        self.cmd.close(0)
        self.ctx.term()


def run_profile(link, kp, ki, accel, label):
    print(f"  {label}: kp={kp} ki={ki}")
    data = []
    for v, secs in PROFILE:
        seg = link.run({"mode": "speed", "left": v, "right": v, "kp": kp, "ki": ki, "accel": accel}, secs)
        for s in seg:
            s["target"] = v
        data += seg
    link.run({"mode": "speed", "left": 0, "right": 0}, 1.0)
    return data


def metrics(data, t0):
    """Steady-state error and noise for each constant-speed segment (last 1.5 s)."""
    rows, t_seg = [], 0.0
    for v, secs in PROFILE:
        if v > 0:
            seg = [s for s in data if t_seg + secs - 1.5 <= s["t"] - t0 < t_seg + secs]
            r = {"target": v}
            for side in ("l", "r"):
                vals = [s[f"v_{side}"] for s in seg]
                r[f"mean_{side}"] = st.mean(vals) if vals else float("nan")
                r[f"std_{side}"] = st.stdev(vals) if len(vals) > 1 else float("nan")
            rows.append(r)
        t_seg += secs
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--kp", type=float, default=0.8)
    ap.add_argument("--ki", type=float, default=6.0)
    ap.add_argument("--accel", type=float, default=5.0, help="target ramp m/s^2 (5 = nearly a step)")
    args = ap.parse_args()
    LOGDIR.mkdir(exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    link = Link(args.host)

    if not link.run({"mode": "duty", "left": 0, "right": 0}, 1.0):
        sys.exit("No telemetry. Is 'python3 -m pi.io_server' running on the Pi?")
    input("LIFT THE ROBOT so the wheels spin freely, then press Enter...")
    print("Warming up motors for 20 s (keeps results comparable)...")
    link.run({"mode": "duty", "left": 0.7, "right": 0.7}, 20.0)
    link.run({"mode": "duty", "left": 0, "right": 0}, 1.5)

    try:
        print("Running speed profile:")
        results = {
            "open loop": run_profile(link, 0.0, 0.0, args.accel, "OPEN LOOP  (feedforward only)"),
            "closed loop": run_profile(link, args.kp, args.ki, args.accel, "CLOSED LOOP (feedforward + PI)"),
        }
    finally:
        link.close()

    # ---- save CSV ----
    f_csv = LOGDIR / f"pid_{stamp}.csv"
    with open(f_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run", "t", "target", "tgt_l", "tgt_r", "v_l", "v_r", "out_l", "out_r", "kp", "ki",
                    "wd_ok", "estop", "wd_age_ms", "dist_cm"])
        for name, data in results.items():
            t0 = data[0]["t"]
            for s in data:
                w.writerow([name, round(s["t"] - t0, 4), s["target"], s["tgt_l"], s["tgt_r"],
                            s["v_l"], s["v_r"], s["out_l"], s["out_r"], s["kp"], s["ki"],
                            s["wd_ok"], s["estop"], s["wd_age_ms"], s["dist_cm"]])

    # ---- safety events during the test ----
    print("\nSafety events during the test:")
    any_event = False
    for name, data in results.items():
        t0 = data[0]["t"]
        prev = None
        for s in data:
            state = ("WATCHDOG" if not s["wd_ok"] else "") + ("OBSTACLE" if s["estop"] else "")
            if state and state != prev:
                print(f"  {name:12s} t={s['t'] - t0:5.2f}s  {state}  "
                      f"(last command {s['wd_age_ms']:.0f} ms ago, ultrasonic {s['dist_cm']:.1f} cm)")
                any_event = True
            prev = state
        gap = max(s["wd_age_ms"] for s in data)
        print(f"  {name:12s} longest gap between commands: {gap:.0f} ms")
    if not any_event:
        print("  none")

    # ---- table ----
    print("\nSteady state (last 1.5 s of each step), m/s")
    print(f"{'run':12s} {'target':>6s} | {'left mean':>9s} {'std':>6s} | {'right mean':>10s} {'std':>6s} | {'L-R':>6s}")
    for name, data in results.items():
        for r in metrics(data, data[0]["t"]):
            print(f"{name:12s} {r['target']:6.2f} | {r['mean_l']:9.3f} {r['std_l']:6.3f} | "
                  f"{r['mean_r']:10.3f} {r['std_r']:6.3f} | {r['mean_l'] - r['mean_r']:+6.3f}")
    print(f"\nSaved logs/{f_csv.name}")

    # ---- plot ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(14, 7), sharex=True, sharey="row")
    for col, (name, data) in enumerate(results.items()):
        t0 = data[0]["t"]
        t = [s["t"] - t0 for s in data]
        ax[0, col].plot(t, [s["target"] for s in data], "k--", lw=1, label="target")
        ax[0, col].plot(t, [s["v_l"] for s in data], label="left")
        ax[0, col].plot(t, [s["v_r"] for s in data], label="right")
        ax[0, col].set(title=f"{name}: wheel speed", ylabel="speed [m/s]")
        ax[1, col].plot(t, [s["out_l"] for s in data], label="left")
        ax[1, col].plot(t, [s["out_r"] for s in data], label="right")
        ax[1, col].set(title=f"{name}: motor duty", xlabel="time [s]", ylabel="PWM duty")
    for a in ax.flat:
        a.grid(True)
        a.legend(fontsize=8)
    fig.suptitle(f"Speed control test (wheels in air)  kp={args.kp}  ki={args.ki}")
    fig.tight_layout()
    png = LOGDIR / f"pid_{stamp}.png"
    fig.savefig(png, dpi=120)
    print(f"Plot saved: logs/{png.name}")
    subprocess.Popen(["xdg-open", str(png)])


if __name__ == "__main__":
    main()
