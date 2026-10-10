import lgpio, time

PINS = [2,3,4,5,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22,23,24,25,26,27]
h = lgpio.gpiochip_open(0)
last, count = {}, {}
for p in PINS:
    lgpio.gpio_claim_input(h, p)
    last[p] = lgpio.gpio_read(h, p)
    count[p] = 0

print("Do ONE action at a time. Ctrl+C to stop.")
try:
    t0 = time.time()
    while True:
        for p in PINS:
            v = lgpio.gpio_read(h, p)
            if v != last[p]:
                count[p] += 1
                last[p] = v
        if time.time() - t0 >= 1:
            active = {f"GPIO{p}": f"{c} changes (now {last[p]})" for p, c in count.items() if c}
            if active:
                print(active)
            count = {p: 0 for p in PINS}
            t0 = time.time()
        time.sleep(0.001)
except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
