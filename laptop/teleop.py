#!/usr/bin/env python3
"""
sentinel-pi | laptop/teleop.py   (Phase 2: runs on the laptop)

Drive the robot with the keyboard, watch live sensor data, and log every
telemetry sample to a CSV file in logs/ (used later for PID tuning and plots).

Run:
    python3 teleop.py --host <PI_IP>

Keys (click on the window first):
    W / S    speed up / slow down   (negative = backward)
    A / D    steer left / right
    SPACE    stop (speed and steer = 0)
    R        reset the encoder counters shown on screen
    Q / ESC  quit
Commands are sent 20x per second. If this program stops, the Pi's watchdog
stops the motors within 0.3 s.
"""
import argparse
import csv
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import zmq

FIELDS = ["t_recv", "seq", "t", "loop_ms", "enc_l", "enc_r", "dl", "dr", "pos_l", "pos_r",
          "dist_cm", "ir_l", "ir_r", "cmd_l", "cmd_r", "out_l", "out_r", "wd_ok", "estop"]
SPEED_STEP, STEER_STEP = 0.1, 0.1
GREEN, RED, YELLOW, WHITE, GREY = (80, 220, 80), (60, 60, 230), (0, 210, 230), (235, 235, 235), (150, 150, 150)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="Raspberry Pi IP, e.g. 10.45.2.233")
    ap.add_argument("--max", type=float, default=0.8, help="max motor duty (0..1)")
    ap.add_argument("--no-log", action="store_true", help="do not write a CSV log")
    args = ap.parse_args()

    ctx = zmq.Context()
    tel = ctx.socket(zmq.SUB)
    tel.setsockopt(zmq.SUBSCRIBE, b"")
    tel.connect(f"tcp://{args.host}:5556")
    cmd = ctx.socket(zmq.PUB)
    cmd.setsockopt(zmq.SNDHWM, 1)
    cmd.connect(f"tcp://{args.host}:5557")

    writer, logfile = None, None
    if not args.no_log:
        logdir = Path(__file__).resolve().parent.parent / "logs"
        logdir.mkdir(exist_ok=True)
        path = logdir / f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        logfile = open(path, "w", newline="")
        writer = csv.DictWriter(logfile, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        print(f"[teleop] logging to {path}")

    speed = steer = 0.0
    last = None                      # newest telemetry sample
    last_rx = 0.0
    enc_offset = (0, 0)
    loop_hist = deque(maxlen=100)    # Pi loop periods (ms)
    rx_times = deque(maxlen=50)      # arrival times, for telemetry rate
    vel_l = vel_r = 0.0              # smoothed counts/s

    print("[teleop] W/S speed, A/D steer, SPACE stop, R reset counters, Q quit")
    try:
        while True:
            # ---- receive all waiting telemetry ----
            while True:
                try:
                    s = json.loads(tel.recv(zmq.NOBLOCK))
                except zmq.Again:
                    break
                s["t_recv"] = time.time()
                last, last_rx = s, time.monotonic()
                rx_times.append(last_rx)
                loop_hist.append(s["loop_ms"])
                dt = max(s["loop_ms"], 1.0) / 1000.0
                vel_l = 0.8 * vel_l + 0.2 * (s["dl"] / dt)
                vel_r = 0.8 * vel_r + 0.2 * (s["dr"] / dt)
                if writer:
                    writer.writerow(s)

            # ---- keyboard ----
            key = cv2.waitKey(50) & 0xFF          # ~20 Hz loop
            if key in (ord("q"), 27):
                break
            elif key == ord("w"):
                speed += SPEED_STEP
            elif key == ord("s"):
                speed -= SPEED_STEP
            elif key == ord("a"):
                steer += STEER_STEP
            elif key == ord("d"):
                steer -= STEER_STEP
            elif key == ord(" "):
                speed = steer = 0.0
            elif key == ord("r") and last:
                enc_offset = (last["enc_l"], last["enc_r"])
            speed = round(max(-args.max, min(args.max, speed)), 2)
            steer = round(max(-args.max, min(args.max, steer)), 2)

            # differential drive mixing: steer > 0 turns left (right wheel faster)
            left = max(-args.max, min(args.max, speed - steer))
            right = max(-args.max, min(args.max, speed + steer))
            cmd.send_string(json.dumps({"left": left, "right": right}))   # also the heartbeat

            # ---- draw dashboard ----
            img = np.full((420, 560, 3), 30, np.uint8)
            link = last is not None and (time.monotonic() - last_rx) < 0.5

            def put(text, y, color=WHITE, scale=0.6, x=15):
                cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

            put("SENTINEL-PI  teleop", 30, WHITE, 0.8)
            put("LINK OK" if link else "NO LINK", 30, GREEN if link else RED, 0.7, 420)
            put(f"command  speed {speed:+.1f}  steer {steer:+.1f}  ->  L {left:+.2f}  R {right:+.2f}", 70)

            if last:
                if not last["wd_ok"]:
                    put("PI WATCHDOG: motors stopped", 100, RED)
                elif last["estop"]:
                    put(f"OBSTACLE STOP: forward blocked (< 15 cm)", 100, RED)
                else:
                    put("motors active", 100, GREEN)

                d = last["dist_cm"]
                put(f"ultrasonic   {d:6.1f} cm", 145, RED if d < 15 else (YELLOW if d < 40 else WHITE))
                bar = int(min(d, 200) / 200 * 300)
                cv2.rectangle(img, (230, 132), (230 + bar, 148), YELLOW, -1)
                cv2.rectangle(img, (230, 132), (530, 148), GREY, 1)

                put(f"IR  left {'DETECT' if last['ir_l'] else '  -   '}   right {'DETECT' if last['ir_r'] else '  -'}",
                    180, YELLOW if (last["ir_l"] or last["ir_r"]) else WHITE)
                put(f"encoder L  {last['enc_l'] - enc_offset[0]:7d} counts   {vel_l:7.0f} counts/s", 220)
                put(f"encoder R  {last['enc_r'] - enc_offset[1]:7d} counts   {vel_r:7.0f} counts/s", 250)

                rate = (len(rx_times) - 1) / (rx_times[-1] - rx_times[0]) if len(rx_times) > 1 and rx_times[-1] > rx_times[0] else 0
                arr = np.array(loop_hist)
                put(f"telemetry {rate:5.1f} Hz   Pi loop mean {arr.mean():5.1f} ms  max {arr.max():5.1f} ms", 295, GREY, 0.5)
            else:
                put(f"waiting for io_server on {args.host} ...", 145, GREY)

            put("W/S speed  A/D steer  SPACE stop  R reset counters  Q quit", 395, GREY, 0.5)
            cv2.imshow("sentinel-pi teleop", img)
    except KeyboardInterrupt:
        pass
    finally:
        cmd.send_string(json.dumps({"left": 0.0, "right": 0.0}))
        time.sleep(0.1)
        cv2.destroyAllWindows()
        if logfile:
            logfile.close()
        tel.close(0)
        cmd.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
