#!/usr/bin/env python3
"""
sentinel-pi | laptop/line_follow.py   (line following, step 2: closed-loop steering)

Camera frame -> line detection (line_detect.py, saved settings) -> steering -> wheel
speeds (speed mode) -> Pi. The Pi's safety rules stay active: watchdog (laptop silent
for 0.3 s -> stop) and obstacle stop (< 15 cm).

    steering  omega = -(kp_off * offset + kp_ang * angle)        [rad/s, + = turn left]
    wheels    left  = v - omega * track / 2
              right = v + omega * track / 2
    speed     v = base speed, reduced in curves (large |angle|)
    line lost for > 0.3 s, or no camera frame for > 0.3 s  ->  STOP

Needs on the Pi:  python3 pi/camera_server.py   AND   python3 -m pi.io_server
    python3 line_follow.py --host raspberrypi.local

Keys (click the video window):  SPACE start / pause    Q quit
Sliders (window "tuning"): speed, kp_off, kp_ang - change them live while driving.
Every run is logged to logs/linefollow_<time>.csv
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.robot import TRACK_WIDTH_M, V_MAX, V_MIN  # noqa: E402
from line_detect import detect_line, load_params      # noqa: E402

LOGDIR = Path(__file__).resolve().parent.parent / "logs"
LOST_S = 0.3          # stop if the line (or the camera) is gone for this long
SMOOTH = 0.5          # low-pass on offset/angle (0 = none, closer to 1 = smoother but slower)


def wheel_speeds(v, omega):
    """Turn (v, omega) into wheel speeds the Pi controller can actually drive."""
    left = v - omega * TRACK_WIDTH_M / 2
    right = v + omega * TRACK_WIDTH_M / 2
    out = []
    for w in (left, right):
        w = max(-V_MAX, min(V_MAX, w))
        if 0 <= w < V_MIN:            # below the minimum steady speed:
            w = V_MIN if w >= V_MIN / 2 else 0.0   # either crawl at V_MIN or stop (pivot)
        out.append(w)
    return out[0], out[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    args = ap.parse_args()

    ctx = zmq.Context()
    cam = ctx.socket(zmq.SUB)
    cam.setsockopt(zmq.RCVHWM, 2)
    cam.setsockopt(zmq.SUBSCRIBE, b"")
    cam.connect(f"tcp://{args.host}:5555")
    tel = ctx.socket(zmq.SUB)
    tel.setsockopt(zmq.SUBSCRIBE, b"")
    tel.connect(f"tcp://{args.host}:5556")
    cmd = ctx.socket(zmq.PUB)
    cmd.setsockopt(zmq.SNDHWM, 1)
    cmd.connect(f"tcp://{args.host}:5557")

    p = load_params()
    print(f"[line_follow] detection settings: {p}")

    cv2.namedWindow("tuning", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("tuning", 700, 220)
    cv2.createTrackbar("speed cm/s", "tuning", 19, int(V_MAX * 100), lambda v: None)
    cv2.createTrackbar("kp_off x10", "tuning", 9, 60, lambda v: None)
    cv2.createTrackbar("kp_ang x10", "tuning", 5, 60, lambda v: None)
    # some OpenCV/Qt builds ignore the start value -> set it explicitly
    # defaults tuned on the floor 2026-10-28 (straight line): 0.19 m/s, kp_off 0.9, kp_ang 0.5
    cv2.setTrackbarPos("speed cm/s", "tuning", 19)
    cv2.setTrackbarPos("kp_off x10", "tuning", 9)
    cv2.setTrackbarPos("kp_ang x10", "tuning", 5)

    LOGDIR.mkdir(exist_ok=True)
    logpath = LOGDIR / f"linefollow_{datetime.now():%Y%m%d_%H%M%S}.csv"
    logf = open(logpath, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "running", "found", "offset", "angle_deg", "v", "omega", "left", "right",
                  "v_l_meas", "v_r_meas", "estop", "proc_ms"])
    print(f"[line_follow] logging to {logpath}")
    print("[line_follow] SPACE = start / pause, Q = quit")

    running = False
    off_f = ang_f = 0.0
    last_seen = 0.0
    last_frame_t = time.monotonic()
    telem = None
    left = right = 0.0
    frame = None
    last_gains = None
    t0 = time.monotonic()

    try:
        while True:
            # ---- newest camera frame (non-blocking, so commands keep flowing) ----
            got = None
            while cam.poll(0):
                got = cam.recv_multipart()
            if got is None and cam.poll(30):
                got = cam.recv_multipart()
            while tel.poll(0):
                telem = json.loads(tel.recv())

            now = time.monotonic()
            res = None
            proc_ms = 0.0
            if got is not None:
                frame = cv2.imdecode(np.frombuffer(got[2], np.uint8), cv2.IMREAD_COLOR)
                last_frame_t = now
                tp = time.perf_counter()
                res, _, _, pts = detect_line(frame, p)
                proc_ms = (time.perf_counter() - tp) * 1000
                if res:
                    off_f = SMOOTH * off_f + (1 - SMOOTH) * res["offset"]
                    ang_f = SMOOTH * ang_f + (1 - SMOOTH) * math.radians(res["angle"])
                    last_seen = now

            base = max(V_MIN, cv2.getTrackbarPos("speed cm/s", "tuning") / 100)   # never below V_MIN
            kp_off = cv2.getTrackbarPos("kp_off x10", "tuning") / 10
            kp_ang = cv2.getTrackbarPos("kp_ang x10", "tuning") / 10

            gains = (int(base * 100), int(kp_off * 10), int(kp_ang * 10))
            if gains != last_gains:
                print(f"[line_follow] speed {gains[0]} cm/s   kp_off x10 = {gains[1]}   kp_ang x10 = {gains[2]}")
                last_gains = gains

            line_ok = (now - last_seen) < LOST_S
            cam_ok = (now - last_frame_t) < LOST_S
            v = omega = 0.0
            if running and line_ok and cam_ok:
                omega = -(kp_off * off_f + kp_ang * ang_f)
                v = max(V_MIN, base * (1 - 0.5 * min(abs(ang_f) / math.radians(30), 1)))
                left, right = wheel_speeds(v, omega)
            else:
                left = right = 0.0
            cmd.send_string(json.dumps({"mode": "speed", "left": left, "right": right}))

            if got is not None:
                log.writerow([round(now - t0, 3), int(running), int(res is not None),
                              round(res["offset"], 4) if res else "", round(res["angle"], 2) if res else "",
                              round(v, 3), round(omega, 3), round(left, 3), round(right, 3),
                              telem["v_l"] if telem else "", telem["v_r"] if telem else "",
                              int(telem["estop"]) if telem else "", round(proc_ms, 2)])

            # ---- display ----
            if frame is not None and got is not None:
                vis = frame.copy()
                h, w = vis.shape[:2]
                y0 = int(h * p["roi_top"] / 100)
                cv2.rectangle(vis, (0, y0), (w - 1, h - 1), (255, 200, 0), 1)
                cv2.line(vis, (w // 2, y0), (w // 2, h), (120, 120, 120), 1)
                if res:
                    cv2.line(vis, (int(res["a"] * y0 + res["b"]), y0), (int(res["a"] * h + res["b"]), h),
                             (0, 255, 0), 2)
                if not running:
                    state, col = "PAUSED - press SPACE to start", (0, 210, 230)
                elif telem and telem["estop"]:
                    state, col = "OBSTACLE STOP", (0, 0, 255)
                elif not cam_ok:
                    state, col = "NO CAMERA -> STOP", (0, 0, 255)
                elif not line_ok:
                    state, col = "LINE LOST -> STOP", (0, 0, 255)
                else:
                    state, col = "FOLLOWING", (0, 255, 0)
                cv2.putText(vis, state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
                cv2.putText(vis, f"offset {off_f:+.2f}  angle {math.degrees(ang_f):+5.1f}  "
                                 f"L {left:.2f} R {right:.2f} m/s",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(vis, f"speed {base * 100:.0f} cm/s   kp_off {kp_off:.1f}   kp_ang {kp_ang:.1f}",
                            (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 210, 230), 2)
                if telem:
                    cv2.putText(vis, f"measured L {telem['v_l']:+.2f} R {telem['v_r']:+.2f} m/s   "
                                     f"ultrasonic {telem['dist_cm']:.0f} cm",
                                (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                else:
                    cv2.putText(vis, "no telemetry - is io_server running?", (10, 78),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.imshow("line follow", vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                running = not running
                print("[line_follow]", "RUNNING" if running else "PAUSED")
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            cmd.send_string(json.dumps({"mode": "speed", "left": 0.0, "right": 0.0}))
            time.sleep(0.05)
        logf.close()
        cv2.destroyAllWindows()
        for s in (cam, tel, cmd):
            s.close(0)
        ctx.term()
        if last_gains:
            print(f"[line_follow] final values: speed {last_gains[0]}, kp_off x10 {last_gains[1]}, "
                  f"kp_ang x10 {last_gains[2]}")
        print(f"[line_follow] stopped. Log: {logpath}")


if __name__ == "__main__":
    main()
