"""Live object detection on Raspberry Pi 3B with MobileNet-SSD.
View in a browser: http://<pi-ip>:8000
"""
import threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from picamera2 import Picamera2
from libcamera import Transform

PROTO = "models/MobileNetSSD_deploy.prototxt"
MODEL = "models/MobileNetSSD_deploy.caffemodel"
CONF_THRESHOLD = 0.5
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle",
           "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
           "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
           "tvmonitor"]

net = cv2.dnn.readNetFromCaffe(PROTO, MODEL)

# Camera is mounted upside down -> rotate 180°
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    transform=Transform(hflip=1, vflip=1)))
picam2.start()
time.sleep(1)

latest_jpeg = None
lock = threading.Lock()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    frame = latest_jpeg
                if frame:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame + b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


server = ThreadingHTTPServer(("0.0.0.0", 8000), StreamHandler)
threading.Thread(target=server.serve_forever, daemon=True).start()
print("Open http://<pi-ip>:8000 in your laptop browser. Ctrl+C to stop.")

try:
    t_prev = time.time()
    while True:
        frame = picam2.capture_array()          # BGR, 640x480
        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                     0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()

        found = []
        for i in range(detections.shape[2]):
            conf = float(detections[0, 0, i, 2])
            if conf < CONF_THRESHOLD:
                continue
            label = CLASSES[int(detections[0, 0, i, 1])]
            x1, y1, x2, y2 = (detections[0, 0, i, 3:7] * [w, h, w, h]).astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.0%}", (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            found.append(f"{label} {conf:.0%}")

        now = time.time()
        fps = 1 / (now - t_prev)
        t_prev = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest_jpeg = jpg.tobytes()

        if found:
            print(f"[{fps:.1f} FPS] " + ", ".join(found))
except KeyboardInterrupt:
    pass
finally:
    server.shutdown()
    picam2.stop()

