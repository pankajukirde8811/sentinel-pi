#!/usr/bin/env python3
"""
sentinel-pi | laptop/camera_viewer.py
Runs on the laptop. Receives JPEG frames from the Pi and shows a live preview.

Install on the laptop (inside a virtual environment):
    python3 -m venv .venv
    source .venv/bin/activate
    pip install opencv-python pyzmq numpy

Run:
    cd ~/Projects/sentinel-pi/laptop
source .venv/bin/activate
python3 camera_viewer.py --host 10.45.2.233
Press 'q' in the video window to quit.
"""
import argparse
import time

import cv2
import numpy as np
import zmq


def receive_latest(sock):
    """Wait for one frame, then drain the queue so we always show the newest frame."""
    msg = sock.recv_multipart()
    while True:
        try:
            msg = sock.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="Raspberry Pi IP address, e.g. 192.168.0.42")
    parser.add_argument("--port", type=int, default=5555)
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.setsockopt(zmq.SUBSCRIBE, b"")  # subscribe to everything
    sock.connect(f"tcp://{args.host}:{args.port}")
    print(f"[viewer] connected to {args.host}:{args.port}, waiting for frames...")

    fps = 0.0
    t_prev = time.time()
    last_id = None
    dropped = 0

    try:
        while True:
            frame_id_b, _ts_b, jpg = receive_latest(sock)
            frame_id = int(frame_id_b)

            # count frames we skipped (network slower than camera)
            if last_id is not None and frame_id > last_id + 1:
                dropped += frame_id - last_id - 1
            last_id = frame_id

            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # smoothed FPS
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-6))
            t_prev = now

            cv2.putText(frame, f"{fps:5.1f} fps  frame {frame_id}  dropped {dropped}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("sentinel-pi camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
