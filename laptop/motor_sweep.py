#!/usr/bin/env python3
"""
sentinel-pi | laptop/motor_sweep.py   (Phase 3, step 1: motor characterization)

Needs pi/io_server.py running on the Pi. LIFT THE ROBOT: wheels must spin freely.

Test 1 - duty sweep : runs both motors at 10%..100% duty, measures steady speed.
                      -> deadband (minimum duty that moves the wheel)
                      -> left/right mismatch, max speed
Test 2 - step       : 0 -> 60% duty, records the speed-up curve.
                      -> time constant, used to design the PID in Phase 3

Run:
    pip install matplotlib
    python3 motor_sweep.py --host 10.45.2.233
"""
import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.robot import METRES_PER_COUNT  # noqa: E402

LOGDIR = Path(__file__).resolve().parent.parent / "logs"


class Link:
    def __init__(self, host):
        self.ctx = zmq.Context()
        self.tel = self.ctx.socket(zmq.SUB)
        self.tel.setsockopt(zmq.SUBSCRIBE, b"")
        self.tel.connect(f"tcp://{host}:5556")
        self.cmd = self.ctx.socket(zmq.PUB)
        self.cmd.connect(f"tcp://{host}:5557")

    def run(self, left, right, seconds):
        """Send a constant command for `seconds`; return all telemetry received."""
        samples, t_end = [], time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.cmd.send_string(json.dumps({"left": left, "right": right}))
            t_next = time.monotonic() + 0.05
            while time.monotonic() < t_next:
                if self.tel.poll(5):
                    samples.append(json.loads(self.tel.recv()))
        return samples

    def close(self):
        self.run(0.0, 0.0, 0.3)
        self.tel.close(0)
        self.cmd.close(0)
        self.ctx.term()


def speed_mps(samples, key):
    """Average speed from first/last sample (Pi timestamps), in m/s."""
    if len(samples) < 2:
        return float("nan")
    dc = samples[-1][key] - samples[0][key]
    dt = samples[-1]["t"] - samples[0]["t"]
    return dc * METRES_PER_COUNT / dt if dt > 0 else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    args = ap.parse_args()
    LOGDIR.mkdir(exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    link = Link(args.host)

    print("Waiting for telemetry...")
    if not link.run(0, 0, 1.0):
        sys.exit("No telemetry. Is 'python3 -m pi.io_server' running on the Pi?")
    input("LIFT THE ROBOT so the wheels spin freely, then press Enter...")

    try:
        # ---------- Test 1: duty sweep ----------
        print("\nTest 1: duty sweep")
        print(" duty |  left m/s | right m/s")
        sweep = []
        for i in range(1, 11):
            duty = i / 10
            s = link.run(duty, duty, 2.5)
            steady = [x for x in s if x["t"] >= s[-1]["t"] - 1.5] if s else []   # last 1.5 s
            vl, vr = speed_mps(steady, "enc_l"), speed_mps(steady, "enc_r")
            sweep.append({"duty": duty, "v_left": vl, "v_right": vr})
            print(f" {duty:4.1f} | {vl:9.3f} | {vr:9.3f}")
        link.run(0, 0, 1.5)

        f1 = LOGDIR / f"sweep_{stamp}.csv"
        with open(f1, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["duty", "v_left", "v_right"])
            w.writeheader()
            w.writerows(sweep)

        # ---------- Test 2: step response ----------
        print("\nTest 2: step 0 -> 0.6")
        pre = link.run(0, 0, 0.5)
        step = link.run(0.6, 0.6, 2.0)
        post = link.run(0, 0, 1.5)
        allx = pre + step + post
        t0 = step[0]["t"] if step else 0
        f2 = LOGDIR / f"step_{stamp}.csv"
        with open(f2, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "duty_l", "duty_r", "enc_l", "enc_r"])
            for x in allx:
                w.writerow([round(x["t"] - t0, 4), x["out_l"], x["out_r"], x["enc_l"], x["enc_r"]])
        print(f"\nSaved {f1.name} and {f2.name} in logs/")
    finally:
        link.close()

    # ---------- plots ----------
    try:
        import matplotlib
        matplotlib.use("Agg")            # draw to file only; no GUI backend needed
        import matplotlib.pyplot as plt
    except ImportError:
        print("Install matplotlib for plots:  pip install matplotlib")
        return

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    d = [r["duty"] for r in sweep]
    ax[0].plot(d, [r["v_left"] for r in sweep], "o-", label="left")
    ax[0].plot(d, [r["v_right"] for r in sweep], "s-", label="right")
    ax[0].set(title="Duty sweep (wheels in air)", xlabel="PWM duty", ylabel="speed [m/s]")
    ax[0].grid(True)
    ax[0].legend()

    # speed over 100 ms windows (raw 20 ms counts are too coarse)
    win = 5
    for key, lab in (("enc_l", "left"), ("enc_r", "right")):
        ts, vs = [], []
        for i in range(win, len(allx)):
            dt = allx[i]["t"] - allx[i - win]["t"]
            if dt > 0:
                ts.append(allx[i]["t"] - t0)
                vs.append((allx[i][key] - allx[i - win][key]) * METRES_PER_COUNT / dt)
        ax[1].plot(ts, vs, label=lab)
    ax[1].axvline(0, color="grey", ls="--", lw=0.8)
    ax[1].set(title="Step response 0 -> 0.6 duty", xlabel="time [s]", ylabel="speed [m/s]")
    ax[1].grid(True)
    ax[1].legend()
    fig.tight_layout()
    png = LOGDIR / f"motor_characterization_{stamp}.png"
    fig.savefig(png, dpi=120)
    print(f"Plot saved: logs/{png.name}")
    import subprocess
    subprocess.Popen(["xdg-open", str(png)])   # open in the default image viewer


if __name__ == "__main__":
    main()
