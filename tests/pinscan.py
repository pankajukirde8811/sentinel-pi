import lgpio, time

# All usable GPIOs except 14/15 (serial, reserved for HC-05)
PINS = [2,3,4,5,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22,23,24,25,26,27]

h = lgpio.gpiochip_open(0)
last = {}
for p in PINS:
    lgpio.gpio_claim_input(h, p)
    last[p] = lgpio.gpio_read(h, p)

print("Watching pins... trigger a sensor. Ctrl+C to stop.")
print("Start states:", last)
try:
    while True:
        for p in PINS:
            v = lgpio.gpio_read(h, p)
            if v != last[p]:
                print(f"GPIO{p}: {last[p]} -> {v}")
                last[p] = v
        time.sleep(0.005)
except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
