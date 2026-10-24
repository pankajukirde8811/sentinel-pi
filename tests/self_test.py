"""Sentinel-Pi hardware self-test. Run with the car lifted."""
from time import sleep
from gpiozero import DistanceSensor, DigitalInputDevice, LED, Button
from config.pins import ULTRASONIC, ENCODERS, LED as LED_PIN, BUTTON
from robot.drive import Drive

results = {}


def ask(msg):
    return input(f"{msg} [y/n]: ").strip().lower() == "y"


# 1. Camera
print("\n[1] Camera")
try:
    from picamera2 import Picamera2
    cam = Picamera2()
    cam.start()
    sleep(1)
    frame = cam.capture_array()
    cam.stop()
    cam.close()
    results["Camera"] = frame is not None and frame.size > 0
except Exception as e:
    print("  error:", e)
    results["Camera"] = False

# 2. Ultrasonic
print("\n[2] Ultrasonic - hold your hand ~10 cm in front, then remove it")
us = DistanceSensor(echo=ULTRASONIC["echo"], trigger=ULTRASONIC["trigger"],
                    max_distance=2)
readings = []
for _ in range(10):
    d = us.distance * 100
    readings.append(d)
    print(f"  {d:.1f} cm")
    sleep(0.4)
us.close()
results["Ultrasonic"] = max(readings) - min(readings) > 5

# 3. Encoders
for name, pin in ENCODERS.items():
    print(f"\n[3] Encoder GPIO{pin} - turn ONE wheel by hand for 5 s")
    enc = DigitalInputDevice(pin)
    n = [0]
    enc.when_activated = lambda: n.__setitem__(0, n[0] + 1)
    sleep(5)
    enc.close()
    print(f"  {n[0]} pulses")
    results[f"Encoder GPIO{pin}"] = n[0] > 5

# 4. LED + button
print("\n[4] LED - watch it blink")
led = LED(LED_PIN)
led.blink(0.3, 0.3, n=3, background=False)
results["LED"] = ask("  Did the LED blink?")
print("  Now press the button within 5 s")
btn = Button(BUTTON)
results["Button"] = btn.wait_for_press(timeout=5)
led.close()
btn.close()

# 5. Motors
print("\n[5] Motors - forward, backward, left, right")
d = Drive(speed=0.7)
for action in (d.forward, d.backward, d.left_turn, d.right_turn):
    action()
    sleep(1)
    d.stop()
    sleep(0.4)
d.close()
results["Motors"] = ask("  Did both wheels move correctly?")

# 6. IR sensors
from config.pins import IR
for side in ("left", "right"):
    ir = Button(IR[side], pull_up=True)
    print(f"\n[6] IR {side.upper()} (GPIO{IR[side]}) - hold paper in front within 5 s")
    detected = ir.wait_for_press(timeout=5)
    print("  obstacle detected" if detected else "  no obstacle seen")
    if detected:
        print("  now remove the paper")
        cleared = ir.wait_for_release(timeout=5)
    else:
        cleared = False
    ir.close()
    results[f"IR {side}"] = bool(detected and cleared)

# Summary
print("\n===== SELF-TEST RESULT =====")
for part, ok in results.items():
    print(f"  {part:20s} {'PASS' if ok else 'FAIL'}")
