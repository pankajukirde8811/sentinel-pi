#!/usr/bin/env python3
"""
sentinel-pi | laptop/line_detect.py   (line following, step 1: detection + tuning)

Shows the live camera image with the detected tape line. Tune the sliders until the
line is found reliably, then press S to save the settings for the line follower.

Needs pi/camera_server.py running on the Pi (io_server is NOT needed for this step).
    python3 line_detect.py --host raspberrypi.local

Sliders (window "controls"):
    line is BRIGHT  1 = tape brighter than floor (silver/white tape), 0 = darker (black tape)
    auto threshold  1 = automatic (Otsu), 0 = use the "threshold" slider
    threshold       brightness limit 0..255 (only when auto = 0)
    ROI top %       only look at the image below this line (the floor in front of the robot)
    min area        ignore blobs smaller than this (noise, dust)
    min contrast    line and floor must differ by at least this much brightness
                    (prevents "finding" a line in plain floor noise)
Keys (click the video window): S save settings, Q quit

Output per frame:
    offset  -1..+1   where the line is at the bottom of the image (+ = line is RIGHT of centre)
    angle   degrees  direction of the line (+ = line turns to the RIGHT further ahead)
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import zmq

PARAMS_FILE = Path(__file__).resolve().parent / "line_params.json"
DEFAULTS = {"bright": 1, "auto": 1, "threshold": 150, "roi_top": 50, "min_area": 300, "min_contrast": 30}
MAX_WIDTH_FRAC = 0.4   # a tape line can't be wider than 40 % of the image
N_BANDS = 6


def load_params():
    try:
        return {**DEFAULTS, **json.loads(PARAMS_FILE.read_text())}
    except (OSError, ValueError):
        return dict(DEFAULTS)


def detect_line(frame, p):
    """Return (result dict or None, mask, debug info). Works on a BGR frame."""
    h, w = frame.shape[:2]
    y0 = int(h * p["roi_top"] / 100)
    roi = frame[y0:, :]
    gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (7, 7), 0)
    mode = cv2.THRESH_BINARY if p["bright"] else cv2.THRESH_BINARY_INV
    if p["auto"]:
        thr, mask = cv2.threshold(gray, 0, 255, mode | cv2.THRESH_OTSU)
    else:
        thr, mask = cv2.threshold(gray, p["threshold"], 255, mode)
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # check 1: is there real contrast between "line" and "floor"?
    on = mask > 0
    if on.sum() == 0 or (~on).sum() == 0:
        return None, mask, thr, []
    contrast = abs(float(gray[on].mean()) - float(gray[~on].mean()))
    if contrast < p["min_contrast"]:
        return None, mask, thr, []

    # centre of the line in several horizontal bands (bottom = closest to the robot)
    pts = []
    bh = mask.shape[0] // N_BANDS
    for i in range(N_BANDS):
        band = mask[i * bh:(i + 1) * bh, :]
        n, _, stats, cents = cv2.connectedComponentsWithStats(band)
        best = None
        for j in range(1, n):
            area = stats[j, cv2.CC_STAT_AREA]
            if stats[j, cv2.CC_STAT_WIDTH] > MAX_WIDTH_FRAC * w:   # check 2: too wide for tape
                continue
            if area >= p["min_area"] / N_BANDS and (best is None or area > best[0]):
                best = (area, cents[j][0])
        if best:
            pts.append((best[1], y0 + i * bh + bh / 2))
    if len(pts) < 3:
        return None, mask, thr, pts

    xs, ys = np.array(pts).T
    a, b = np.polyfit(ys, xs, 1)                 # x = a*y + b  (line as function of image row)
    x_bottom = a * h + b
    offset = (x_bottom - w / 2) / (w / 2)        # -1 .. +1
    angle = float(np.degrees(np.arctan(-a)))     # + = line goes to the right further ahead
    return {"offset": float(offset), "angle": angle, "a": a, "b": b, "n_pts": len(pts),
            "contrast": contrast}, mask, thr, pts


def receive_latest(sock):
    msg = sock.recv_multipart()
    while True:
        try:
            msg = sock.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    args = ap.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(f"tcp://{args.host}:5555")

    p = load_params()
    cv2.namedWindow("controls")
    cv2.resizeWindow("controls", 500, 250)
    limits = {"bright": 1, "auto": 1, "threshold": 255, "roi_top": 90, "min_area": 3000, "min_contrast": 120}
    for name, mx in limits.items():
        cv2.createTrackbar(name, "controls", int(p[name]), mx, lambda v: None)

    print("[line_detect] waiting for camera frames ... (S save, Q quit)")
    fps, t_prev = 0.0, time.time()
    while True:
        _, _, jpg = receive_latest(sock)
        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        for name in limits:
            p[name] = cv2.getTrackbarPos(name, "controls")

        t0 = time.perf_counter()
        res, mask, thr, pts = detect_line(frame, p)
        proc_ms = (time.perf_counter() - t0) * 1000
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
        t_prev = now

        h, w = frame.shape[:2]
        y0 = int(h * p["roi_top"] / 100)
        vis = frame.copy()
        cv2.rectangle(vis, (0, y0), (w - 1, h - 1), (255, 200, 0), 1)
        cv2.line(vis, (w // 2, y0), (w // 2, h), (120, 120, 120), 1)
        for x, y in pts:
            cv2.circle(vis, (int(x), int(y)), 5, (0, 0, 255), -1)
        if res:
            ya, yb = y0, h
            cv2.line(vis, (int(res["a"] * ya + res["b"]), ya), (int(res["a"] * yb + res["b"]), yb),
                     (0, 255, 0), 2)
            txt = f"offset {res['offset']:+.2f}   angle {res['angle']:+5.1f} deg   contrast {res['contrast']:.0f}"
            col = (0, 255, 0)
        else:
            txt, col = "LINE NOT FOUND", (0, 0, 255)
        cv2.putText(vis, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(vis, f"{fps:4.1f} fps  processing {proc_ms:4.1f} ms  threshold {thr:.0f}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # mask preview in the top-right corner
        small = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (w // 3, (h - y0) // 3 or 1))
        vis[0:small.shape[0], w - small.shape[1]:w] = small
        cv2.imshow("line detect", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            PARAMS_FILE.write_text(json.dumps(p, indent=2))
            print(f"[line_detect] saved {p} -> {PARAMS_FILE.name}")

    cv2.destroyAllWindows()
    sock.close(0)
    ctx.term()


if __name__ == "__main__":
    main()
