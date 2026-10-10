from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=27, trigger=25, max_distance=2)

try:
    while True:
        print(f"Distance: {sensor.distance * 100:.1f} cm")
        sleep(0.5)
except KeyboardInterrupt:
    pass
