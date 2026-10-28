#!/usr/bin/env python3
"""
sentinel-pi | laptop/manual_calibrate.py   (distance calibration by hand driving)

You drive the robot yourself exactly 2.00 m along the tape line and stop with the
wheel axle on the 2 m mark. The script counts the encoder pulses on the way and
calculates the real distance per encoder count.

Keys (click on the window first):
    hold W         drive forward   (release = stop)
    hold S         drive backward  (for small corrections if you went too far)
    hold A / D     while W or S is held: steer left / right (keep on the line)
    ENTER          robot is exactly on the 2 m mark -> calculate and save
    R              new run: robot is back on the start line -> counters to zero
    Q / ESC        quit

Needs pi/io_server.py running on the Pi.
    pip install pygame-ce
    python3 manual_calibrate.py --host raspberrypi.local
Results are appended to logs/manual_calibration.csv
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import pygame
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.robot import COUNTS_PER_REV, METRES_PER_COUNT  # noqa: E402

LOGDIR = Path(__file__).resolve().parent.parent / "logs"
SPEED = 0.22      # m/s while a key is held (minimum steady speed is ~0.18)
STEER = 0.04      # m/s difference between the wheels when steering


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--dist", type=float, default=2.0, help="real distance to the mark [m]")
    args = ap.parse_args()

    ctx = zmq.Context()
    tel = ctx.socket(zmq.SUB)
    tel.setsockopt(zmq.SUBSCRIBE, b"")
    tel.connect(f"tcp://{args.host}:5556")
    cmd = ctx.socket(zmq.PUB)
    cmd.setsockopt(zmq.SNDHWM, 1)
    cmd.connect(f"tcp://{args.host}:5557")

    pygame.init()
    screen = pygame.display.set_mode((680, 340))
    pygame.display.set_caption("sentinel-pi manual calibration")
    font = pygame.font.SysFont("monospace", 18)
    big = pygame.font.SysFont("monospace", 26, bold=True)
    clock = pygame.time.Clock()

    last = None
    start = None            # (pos_l, pos_r) at the start line
    results = []
    message = "Place the robot on the START LINE, then press R."
    running = True

    while running:
        # ---- telemetry ----
        while tel.poll(0):
            last = json.loads(tel.recv())
        if last is not None and start is None:
            start = (last["pos_l"], last["pos_r"])

        # ---- keys ----
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif ev.key == pygame.K_r and last:
                    start = (last["pos_l"], last["pos_r"])
                    message = "Counters at zero. Hold W to drive to the 2 m mark."
                elif ev.key == pygame.K_RETURN and last and start:
                    # wait until the robot stands still, then read the counts
                    t_end = time.monotonic() + 1.0
                    while time.monotonic() < t_end:
                        cmd.send_string(json.dumps({"mode": "speed", "left": 0, "right": 0}))
                        while tel.poll(20):
                            last = json.loads(tel.recv())
                    cl = last["pos_l"] - start[0]
                    cr = last["pos_r"] - start[1]
                    mean = (cl + cr) / 2
                    if mean < 50:
                        message = "Too few counts. Drive to the 2 m mark first."
                        continue
                    mpc = args.dist / mean
                    cpr = COUNTS_PER_REV * METRES_PER_COUNT / mpc
                    results.append((cl, cr, mpc, cpr))
                    LOGDIR.mkdir(exist_ok=True)
                    f = LOGDIR / "manual_calibration.csv"
                    new = not f.exists()
                    with open(f, "a", newline="") as fh:
                        w = csv.writer(fh)
                        if new:
                            w.writerow(["time", "real_dist_m", "counts_l", "counts_r",
                                        "mm_per_count", "counts_per_rev"])
                        w.writerow([f"{datetime.now():%Y-%m-%d %H:%M:%S}", args.dist, cl, cr,
                                    round(mpc * 1000, 4), round(cpr, 3)])
                    message = (f"Run {len(results)} saved: {mpc * 1000:.3f} mm/count. "
                               f"Back to start, press R.")
                    start = (last["pos_l"], last["pos_r"])

        keys = pygame.key.get_pressed()
        v = SPEED if keys[pygame.K_w] else (-SPEED if keys[pygame.K_s] else 0.0)
        steer = 0.0
        if v != 0.0:
            if keys[pygame.K_a]:
                steer = STEER
            elif keys[pygame.K_d]:
                steer = -STEER
        # A always turns the robot's nose LEFT, D always RIGHT (also when driving backward)
        left, right = v - steer, v + steer
        cmd.send_string(json.dumps({"mode": "speed", "left": left, "right": right}))

        # ---- draw ----
        screen.fill((30, 30, 30))

        def put(text, y, color=(235, 235, 235), f=font):
            screen.blit(f.render(text, True, color), (15, y))

        put("MANUAL DISTANCE CALIBRATION", 12, (235, 235, 235), big)
        if last is None:
            put("waiting for io_server ...", 60, (230, 80, 80))
        else:
            cl = last["pos_l"] - start[0] if start else 0
            cr = last["pos_r"] - start[1] if start else 0
            mean = (cl + cr) / 2
            put(f"counts   left {cl:5d}   right {cr:5d}   mean {mean:7.1f}", 60)
            put(f"distance with current calibration: {mean * METRES_PER_COUNT:6.3f} m", 90)
            state = "OBSTACLE STOP" if last["estop"] else ("WATCHDOG STOP" if not last["wd_ok"] else "")
            if state:
                put(state, 120, (230, 80, 80))
            else:
                put(f"command  L {left:+.2f}  R {right:+.2f} m/s", 120, (230, 210, 0))
        put(message, 160, (80, 220, 80))
        for i, (cl, cr, mpc, cpr) in enumerate(results[-3:]):
            put(f"run {len(results) - len(results[-3:]) + i + 1}: L {cl}  R {cr}  ->  "
                f"{mpc * 1000:.3f} mm/count  ({cpr:.2f} counts/turn)", 195 + 24 * i, (150, 150, 150))
        if len(results) >= 2:
            m = sum(r[2] for r in results) / len(results)
            put(f"average of {len(results)} runs: {m * 1000:.3f} mm/count", 270, (230, 210, 0))
        put("hold W/S drive  A/D steer  ENTER on mark  R new run  Q quit", 310, (150, 150, 150))
        pygame.display.flip()
        clock.tick(20)

    for _ in range(5):
        cmd.send_string(json.dumps({"mode": "speed", "left": 0, "right": 0}))
        time.sleep(0.05)
    pygame.quit()
    tel.close(0)
    cmd.close(0)
    ctx.term()


if __name__ == "__main__":
    main()
