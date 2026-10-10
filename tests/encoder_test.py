from gpiozero import DigitalInputDevice
from time import sleep

ENC_A, ENC_B = 23, 16
count = {ENC_A: 0, ENC_B: 0}

def make_counter(pin):
    def inc():
        count[pin] += 1
    return inc

enc_a = DigitalInputDevice(ENC_A)
enc_b = DigitalInputDevice(ENC_B)
enc_a.when_activated = make_counter(ENC_A)
enc_b.when_activated = make_counter(ENC_B)

print("Turn one wheel at a time. Ctrl+C to stop.")
try:
    while True:
        print(f"GPIO23: {count[ENC_A]:5d} pulses | GPIO16: {count[ENC_B]:5d} pulses", end="\r")
        sleep(0.2)
except KeyboardInterrupt:
    print()
