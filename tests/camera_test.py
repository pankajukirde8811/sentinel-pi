from picamera2 import Picamera2
from libcamera import Transform
import time

# Camera is mounted upside down on the chassis -> rotate 180°
picam2 = Picamera2()
config = picam2.create_still_configuration(transform=Transform(hflip=1, vflip=1))
picam2.configure(config)

picam2.start()
time.sleep(2)  # let auto-exposure settle
picam2.capture_file("camera_test.jpg")
picam2.stop()
print("Saved camera_test.jpg")
