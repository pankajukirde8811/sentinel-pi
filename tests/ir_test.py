"""Live IR obstacle sensor test (left = GPIO20, right = GPIO21)."""
from gpiozero import Button
from time import sleep
from config.pins import IR

left = Button(IR["left"], pull_up=True)
right = Button(IR["right"], pull_up=True)

print("Hold paper in front of each IR sensor. Ctrl+C to stop.")
try:
    while True:
        l = "OBSTACLE" if left.is_pressed else "clear   "
        r = "OBSTACLE" if right.is_pressed else "clear   "
        print(f"LEFT: {l}  |  RIGHT: {r}", end="\r")
        sleep(0.1)
except KeyboardInterrupt:
    print()
