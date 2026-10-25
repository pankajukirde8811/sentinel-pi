#!/usr/bin/env python3
"""
sentinel-pi | laptop/floor_test.py   (Phase 3b: floor test + distance calibration)

The robot drives STRAIGHT on the floor in speed mode until its encoders say 2.00 m,
then stops by itself. You measure where it really stopped. The script calculates:
  - distance calibration (real metres per encoder count)
  - left/right wheel size ratio from the sideways drift
Results are appended to logs/floor_tests.csv, so you can repeat it 3x and compare.

Setup: start line + mark at exactly 2.00 m (tape), ~1 m free space after it.
Place the robot with its FRONT WHEEL AXLE on the start line, pointing at the mark.

    python3 floor_test.py --host 10.45.2.233
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
from config.robot import COUNTS_PER_REV, METRES_PER_COUNT, TRACK_WIDTH_M  # noqa: E402

LOGDIR = Path(__file__).resolve().parent.parent / "logs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--dist", type=float, default=2.0, help="encoder distance to drive [m]")
    ap.add_argument("--speed", type=float, default=0.25, help="speed [m/s]")
    args = ap.parse_args()

    ctx = zmq.Context()
    tel = ctx.socket(zmq.SUB)
    tel.setsockopt(zmq.SUBSCRIBE, b"")
    tel.connect(f"tcp://{args.host}:5556")
    cmd = ctx.socket(zmq.PUB)
    cmd.connect(f"tcp://{args.host}:5557")

    def send(v):
        cmd.send_string(json.dumps({"mode": "speed", "left": v, "right": v}))

    def latest(timeout=1.0):
        s, t_end = None, time.monotonic() + timeout
        while time.monotonic() < t_end:
            send(0.0) if s is None else None
            if tel.poll(20):
                s = json.loads(tel.recv())
                while tel.poll(0):
                    s = json.loads(tel.recv())
                return s
        return s

    s0 = latest(2.0)
    if s0 is None:
        sys.exit("No telemetry. Is 'python3 -m pi.io_server' running on the Pi?")
    print("Place the robot on the START LINE, pointing at the 2 m mark.")
    input("Press Enter, then step back. The robot starts after 3 s...")
    for i in (3, 2, 1):
        print(f"  {i}")
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            send(0.0)
            time.sleep(0.05)

    s0 = latest()
    l0, r0 = s0["enc_l"], s0["enc_r"]
    t_start = time.monotonic()
    stopped_by = "distance"
    samples = []
    try:
        while True:
            send(args.speed)
            if tel.poll(50):
                s = json.loads(tel.recv())
                while tel.poll(0):
                    s = json.loads(tel.recv())
                samples.append(s)
                d = ((s["enc_l"] - l0) + (s["enc_r"] - r0)) / 2 * METRES_PER_COUNT
                print(f"\r  driven {d:5.2f} m   v L {s['v_l']:.2f} R {s['v_r']:.2f} m/s", end="")
                if s["estop"]:
                    stopped_by = "OBSTACLE STOP"
                    break
                if not s["wd_ok"] and time.monotonic() - t_start > 1.0:
                    stopped_by = "WATCHDOG"
                    break
                # stop a little early: the 0.8 m/s^2 ramp needs ~4 cm to stop from 0.25 m/s
                if d >= args.dist - args.speed ** 2 / (2 * 0.8):
                    break
            if time.monotonic() - t_start > 3 * args.dist / args.speed:
                stopped_by = "TIMEOUT"
                break
    except KeyboardInterrupt:
        stopped_by = "Ctrl+C"
    finally:
        t_end = time.monotonic() + 1.5          # stop and let it coast to standstill
        while time.monotonic() < t_end:
            send(0.0)
            time.sleep(0.05)
    s1 = latest()
    cl, cr = s1["enc_l"] - l0, s1["enc_r"] - r0
    el, er = cl * METRES_PER_COUNT, cr * METRES_PER_COUNT
    print(f"\n\nStopped by: {stopped_by}")
    print(f"Encoder: left {cl} counts = {el:.3f} m | right {cr} counts = {er:.3f} m | "
          f"mean {(el + er) / 2:.3f} m")
    if stopped_by != "distance":
        print("Test not valid (did not finish normally). Run it again.")
        return

    print("\nNow measure with the tape measure (front wheel axle position):")
    real = float(input("  distance driven FORWARD from the start line [cm]: ")) / 100
    side = float(input("  sideways offset from the straight line [cm] (+ = robot ended LEFT, - = RIGHT): ")) / 100

    mean_counts = (cl + cr) / 2
    mpc_real = real / mean_counts
    cpr_real = COUNTS_PER_REV * METRES_PER_COUNT / mpc_real
    # constant-curvature approximation: lateral offset y = D * theta / 2
    theta = 2 * side / real if real > 0 else 0.0
    ratio = (real + TRACK_WIDTH_M * theta / 2) / (real - TRACK_WIDTH_M * theta / 2)  # right / left

    print("\n===== RESULT =====")
    print(f"  distance error of current calibration: {((el + er) / 2 - real) * 100:+.1f} cm "
          f"({((el + er) / 2 / real - 1) * 100:+.1f} %)")
    print(f"  real metres per count : {mpc_real * 1000:.3f} mm   (config: {METRES_PER_COUNT * 1000:.3f} mm)")
    print(f"  equivalent COUNTS_PER_REV for config/robot.py: {cpr_real:.2f}   (now {COUNTS_PER_REV})")
    print(f"  heading change ~{theta * 57.2958:+.1f} deg -> right/left wheel size ratio ~{ratio:.4f}")

    LOGDIR.mkdir(exist_ok=True)
    f = LOGDIR / "floor_tests.csv"
    new = not f.exists()
    with open(f, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["time", "speed", "counts_l", "counts_r", "enc_dist_m", "real_dist_m",
                        "side_m", "mm_per_count", "counts_per_rev", "heading_deg", "ratio_r_l"])
        w.writerow([f"{datetime.now():%Y-%m-%d %H:%M:%S}", args.speed, cl, cr, round((el + er) / 2, 4),
                    real, side, round(mpc_real * 1000, 4), round(cpr_real, 3),
                    round(theta * 57.2958, 2), round(ratio, 5)])
    print(f"\nSaved to logs/{f.name}. Repeat 3x, then we update config/robot.py with the average.")
    tel.close(0)
    cmd.close(0)
    ctx.term()


if __name__ == "__main__":
    main()
