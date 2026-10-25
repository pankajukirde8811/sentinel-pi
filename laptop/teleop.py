#!/usr/bin/env python3
"""
sentinel-pi | laptop/teleop.py   (runs on the laptop)

Drive the robot with the keyboard, watch live sensor data, and log every
telemetry sample to a CSV file in logs/.

Run:
    python3 teleop.py --host <PI_IP>

Keys (click on the window first):
    M        switch mode: DUTY (raw PWM) <-> SPEED (closed-loop, m/s)
    W / S    faster / slower   (negative = backward)
    A / D    steer left / right
    SPACE    stop
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

FIELDS = ["t_recv", "seq", "t", "loop_ms", "mode", "enc_l", "enc_r", "pos_l", "pos_r",
          "v_l", "v_r", "tgt_l", "tgt_r", "cmd_l", "cmd_r", "out_l", "out_r",
          "dist_cm", "ir_l", "ir_r", "wd_ok", "estop", "glitch_l", "glitch_r", "kp", "ki"]
STEP = {"duty": 0.1, "speed": 0.05}
LIMIT = {"duty": 0.8, "speed": 0.35}
GREEN, RED, YELLOW, WHITE, GREY = (80, 220, 80), (60, 60, 230), (0, 210, 230), (235, 235, 235), (150, 150, 150)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="Raspberry Pi IP, e.g. 10.45.2.233")
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

    mode = "duty"
    speed = steer = 0.0
    last, last_rx = None, 0.0
    enc_offset = (0, 0)
    loop_hist = deque(maxlen=100)
    rx_times = deque(maxlen=50)

    print("[teleop] M mode, W/S speed, A/D steer, SPACE stop, R reset counters, Q quit")
    try:
        while True:
            while True:
                try:
                    s = json.loads(tel.recv(zmq.NOBLOCK))
                except zmq.Again:
                    break
                s["t_recv"] = time.time()
                last, last_rx = s, time.monotonic()
                rx_times.append(last_rx)
                loop_hist.append(s["loop_ms"])
                if writer:
                    writer.writerow(s)

            key = cv2.waitKey(50) & 0xFF
            step, lim = STEP[mode], LIMIT[mode]
            if key in (ord("q"), 27):
                break
            elif key == ord("m"):
                mode = "speed" if mode == "duty" else "duty"
                speed = steer = 0.0
            elif key == ord("w"):
                speed += step
            elif key == ord("s"):
                speed -= step
            elif key == ord("a"):
                steer += step
            elif key == ord("d"):
                steer -= step
            elif key == ord(" "):
                speed = steer = 0.0
            elif key == ord("r") and last:
                enc_offset = (last["enc_l"], last["enc_r"])
            lim = LIMIT[mode]
            speed = round(max(-lim, min(lim, speed)), 2)
            steer = round(max(-lim, min(lim, steer)), 2)
            left = max(-lim, min(lim, speed - steer))
            right = max(-lim, min(lim, speed + steer))
            cmd.send_string(json.dumps({"mode": mode, "left": left, "right": right}))

            img = np.full((440, 600, 3), 30, np.uint8)
            link = last is not None and (time.monotonic() - last_rx) < 0.5

            def put(text, y, color=WHITE, scale=0.6, x=15):
                cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

            unit = "m/s" if mode == "speed" else "duty"
            put("SENTINEL-PI  teleop", 30, WHITE, 0.8)
            put("LINK OK" if link else "NO LINK", 30, GREEN if link else RED, 0.7, 460)
            put(f"mode {mode.upper()} (M)   command L {left:+.2f}  R {right:+.2f} {unit}", 70, YELLOW)

            if last:
                if not last["wd_ok"]:
                    put("PI WATCHDOG: motors stopped", 100, RED)
                elif last["estop"]:
                    put("OBSTACLE STOP: forward blocked (< 15 cm)", 100, RED)
                else:
                    put("motors active", 100, GREEN)
                put(f"speed  L {last['v_l']:+.3f}  R {last['v_r']:+.3f} m/s    "
                    f"duty L {last['out_l']:+.2f}  R {last['out_r']:+.2f}", 135)
                d = last["dist_cm"]
                put(f"ultrasonic   {d:6.1f} cm", 175, RED if d < 15 else (YELLOW if d < 40 else WHITE))
                bar = int(min(d, 200) / 200 * 300)
                cv2.rectangle(img, (240, 162), (240 + bar, 178), YELLOW, -1)
                cv2.rectangle(img, (240, 162), (540, 178), GREY, 1)
                put(f"IR  left {'DETECT' if last['ir_l'] else '  -   '}   right {'DETECT' if last['ir_r'] else '  -'}",
                    210, YELLOW if (last["ir_l"] or last["ir_r"]) else WHITE)
                put(f"encoder L {last['enc_l'] - enc_offset[0]:7d}   R {last['enc_r'] - enc_offset[1]:7d} counts"
                    f"   (R = reset)", 245)
                put(f"glitches filtered  L {last['glitch_l']}  R {last['glitch_r']}", 275, GREY, 0.5)
                rate = (len(rx_times) - 1) / (rx_times[-1] - rx_times[0]) \
                    if len(rx_times) > 1 and rx_times[-1] > rx_times[0] else 0
                arr = np.array(loop_hist)
                put(f"telemetry {rate:5.1f} Hz   Pi loop mean {arr.mean():5.1f} ms  max {arr.max():5.1f} ms",
                    305, GREY, 0.5)
            else:
                put(f"waiting for io_server on {args.host} ...", 135, GREY)

            put("M mode  W/S speed  A/D steer  SPACE stop  R reset  Q quit", 415, GREY, 0.5)
            cv2.imshow("sentinel-pi teleop", img)
    except KeyboardInterrupt:
        pass
    finally:
        cmd.send_string(json.dumps({"mode": mode, "left": 0.0, "right": 0.0}))
        time.sleep(0.1)
        cv2.destroyAllWindows()
        if logfile:
            logfile.close()
        tel.close(0)
        cmd.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
