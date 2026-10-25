#!/usr/bin/env python3
"""
sentinel-pi | pi/camera_server.py
Runs on the Raspberry Pi. Captures camera frames and streams them as JPEG
to the laptop over ZeroMQ (PUB socket).

Install on the Pi:
    sudo apt update
    sudo apt install -y python3-picamera2 python3-opencv python3-zmq

Run:
    python3 camera_viewer.py --host 10.45.2.233
"""
import time

import cv2
import zmq
from libcamera import Transform
from picamera2 import Picamera2

PORT = 5555            # laptop connects to tcp://<PI_IP>:5555
WIDTH, HEIGHT = 640, 480
JPEG_QUALITY = 70      # lower = smaller frames, faster over Wi-Fi
ROTATE_180 = True      # camera is mounted upside down on the robot


def main():
    # --- network ---
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)  # keep max 2 frames queued: drop old frames, never lag behind
    sock.bind(f"tcp://*:{PORT}")

    # --- camera ---
    cam = Picamera2()
    config = cam.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},  # gives BGR order, which OpenCV expects
        transform=Transform(hflip=ROTATE_180, vflip=ROTATE_180),  # hflip + vflip = 180° rotation
    )
    cam.configure(config)
    cam.start()
    time.sleep(1.0)  # let auto-exposure settle

    print(f"[camera_server] streaming {WIDTH}x{HEIGHT} on port {PORT}  (Ctrl+C to stop)")
    frame_id = 0
    t_last = time.time()

    try:
        while True:
            frame = cam.capture_array()
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                continue

            # message = [frame_id, timestamp, jpeg bytes]
            sock.send_multipart([
                str(frame_id).encode(),
                repr(time.time()).encode(),
                jpg.tobytes(),
            ])
            frame_id += 1

            # print FPS every 100 frames
            if frame_id % 100 == 0:
                now = time.time()
                print(f"[camera_server] {100 / (now - t_last):.1f} fps, "
                      f"{len(jpg) / 1024:.0f} kB/frame")
                t_last = now
    except KeyboardInterrupt:
        print("\n[camera_server] stopping")
    finally:
        cam.stop()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
