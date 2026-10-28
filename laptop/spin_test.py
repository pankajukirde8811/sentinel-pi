#!/usr/bin/env python3
"""
sentinel-pi | laptop/spin_test.py   (Phase 5: track width calibration)

The robot spins on the spot for N full turns (counted with the encoders, using the
current TRACK_WIDTH_M), stops by itself, and you measure how far it is from facing
the start direction again. From the real angle and the encoder counts:

    track_width = (distance right wheel - distance left wheel) / real angle

Done once spinning LEFT and once spinning RIGHT; the average cancels direction effects.

Setup:
  - a straight tape line on the floor (~60 cm)
  - a POINTER on the robot: a strip of tape on the robot pointing straight forward,
    its tip sticking out a bit in front (or use a clear mark at the front centre)
  - place the robot so the middle between the wheels is ON the line and the
    pointer tip is exactly above the line

Needs pi/io_server.py running on the Pi.
    python3 spin_test.py --host raspberrypi.local
Results are appended to logs/spin_tests.csv
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.robot import METRES_PER_COUNT, TRACK_WIDTH_M  # noqa: E402

LOGDIR = Path(__file__).resolve().parent.parent / "logs"
ACCEL = 0.8   # m/s^2, the Pi's default ramp (used for the stop allowance)


def ask_number(prompt):
    while True:
        txt = input(prompt).strip().replace(",", ".")
        try:
            return float(txt)
        except ValueError:
            print("    please type a number, e.g. 3.5")


def ask_choice(prompt, options):
    while True:
        txt = input(prompt).strip().lower()
        if txt in options:
            return txt
        print(f"    please type one of: {', '.join(options)}")


class Link:
    def __init__(self, host):
        self.ctx = zmq.Context()
        self.tel = self.ctx.socket(zmq.SUB)
        self.tel.setsockopt(zmq.SUBSCRIBE, b"")
        self.tel.connect(f"tcp://{host}:5556")
        self.cmd = self.ctx.socket(zmq.PUB)
        self.cmd.connect(f"tcp://{host}:5557")
        self.last = None

    def send(self, left, right):
        self.cmd.send_string(json.dumps({"mode": "speed", "left": left, "right": right}))

    def pump(self, wait_ms=50):
        if self.tel.poll(wait_ms):
            while self.tel.poll(0):
                self.last = json.loads(self.tel.recv())
        return self.last

    def hold(self, left, right, seconds):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.send(left, right)
            self.pump(50)
        return self.last

    def close(self):
        self.hold(0, 0, 0.3)
        self.tel.close(0)
        self.cmd.close(0)
        self.ctx.term()


def spin(link, direction, turns, speed):
    """direction +1 = left (counter-clockwise seen from above), -1 = right."""
    s0 = link.hold(0, 0, 1.0)
    l0, r0 = s0["pos_l"], s0["pos_r"]
    target = direction * 2 * math.pi * turns
    # stop early by the angle the wheels need to ramp down: each wheel rolls v^2 / (2a)
    allowance = 2 * (speed ** 2 / (2 * ACCEL)) / TRACK_WIDTH_M
    t_start = time.monotonic()
    while True:
        link.send(-direction * speed, direction * speed)
        s = link.pump(50)
        th = ((s["pos_r"] - r0) - (s["pos_l"] - l0)) * METRES_PER_COUNT / TRACK_WIDTH_M
        print(f"\r  encoder angle {math.degrees(th):7.1f} deg", end="")
        if abs(th) >= abs(target) - allowance:
            break
        if not s["wd_ok"] and time.monotonic() - t_start > 1.0:
            print("\n  WATCHDOG stop - test not valid")
            return None
        if time.monotonic() - t_start > 30:
            print("\n  timeout - test not valid")
            return None
    s1 = link.hold(0, 0, 1.5)           # stop and wait until it stands still
    dl = (s1["pos_l"] - l0) * METRES_PER_COUNT
    dr = (s1["pos_r"] - r0) * METRES_PER_COUNT
    th_enc = (dr - dl) / TRACK_WIDTH_M
    print(f"\r  encoder angle {math.degrees(th_enc):7.1f} deg (after stop)   "
          f"left wheel {dl:+.3f} m, right wheel {dr:+.3f} m")
    return dl, dr, th_enc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--turns", type=int, default=3)
    ap.add_argument("--speed", type=float, default=0.2, help="wheel speed [m/s]")
    args = ap.parse_args()

    link = Link(args.host)
    if link.hold(0, 0, 1.5) is None:
        sys.exit("No telemetry. Is 'python3 -m pi.io_server' running on the Pi?")

    print(f"Current TRACK_WIDTH_M = {TRACK_WIDTH_M * 100:.2f} cm")
    r_cm = ask_number("Distance from the robot centre (middle between the wheels) to the pointer tip [cm]: ")

    results = []
    try:
        for direction, name in ((1, "LEFT (counter-clockwise)"), (-1, "RIGHT (clockwise)")):
            print(f"\n=== Spin {name}, {args.turns} turns ===")
            input("Place the robot: centre on the line, pointer tip exactly above the line. "
                  "Press Enter, then step back...")
            for i in (3, 2, 1):
                print(f"  {i}")
                link.hold(0, 0, 1.0)
            res = spin(link, direction, args.turns, args.speed)
            if res is None:
                continue
            dl, dr, th_enc = res
            print("\nLook at the pointer tip compared to the line:")
            way = ask_choice("  did the robot turn TOO FAR (f), NOT FAR ENOUGH (n), or EXACTLY on the line (e)? ",
                             ("f", "n", "e"))
            d_cm = 0.0 if way == "e" else ask_number("  distance of the pointer tip from the line [cm]: ")
            err = math.atan2(d_cm, r_cm) * (1 if way == "f" else -1)
            th_real = direction * (2 * math.pi * args.turns + err)
            track = (dr - dl) / th_real
            print(f"  real angle {math.degrees(th_real):+.1f} deg  ->  track width {track * 100:.2f} cm")
            results.append((name, dl, dr, th_enc, th_real, track))
    finally:
        link.close()

    if not results:
        return
    LOGDIR.mkdir(exist_ok=True)
    f = LOGDIR / "spin_tests.csv"
    new = not f.exists()
    with open(f, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["time", "direction", "turns", "dist_l_m", "dist_r_m",
                        "enc_angle_deg", "real_angle_deg", "track_width_m"])
        for name, dl, dr, th_enc, th_real, track in results:
            w.writerow([f"{datetime.now():%Y-%m-%d %H:%M:%S}", name.split()[0], args.turns,
                        round(dl, 4), round(dr, 4), round(math.degrees(th_enc), 2),
                        round(math.degrees(th_real), 2), round(track, 5)])
    avg = sum(r[5] for r in results) / len(results)
    print("\n===== RESULT =====")
    for name, *_, track in results:
        print(f"  {name:26s} track width {track * 100:.2f} cm")
    print(f"  average: {avg * 100:.2f} cm   (config now {TRACK_WIDTH_M * 100:.2f} cm)")
    print(f"  -> set TRACK_WIDTH_M = {avg:.4f} in config/robot.py")
    print(f"Saved to logs/{f.name}")


if __name__ == "__main__":
    main()
