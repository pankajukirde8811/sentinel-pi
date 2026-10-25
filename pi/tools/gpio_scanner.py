#!/usr/bin/env python3
"""
sentinel-pi | pi/tools/gpio_scanner.py
Runs on the Raspberry Pi. Watches all GPIO pins (BCM 2-27) as INPUTS and prints
every pin that changes. Use it to discover which pin is connected to which sensor:

    - spin a wheel slowly by hand  -> the encoder pin(s) of that wheel toggle
    - hold a hand / white paper under an IR sensor -> that IR pin toggles

It only READS pins (no output, no pull-up/down changes), so it is safe.
Keep the motor battery switched OFF while running it.

Install (usually already installed on Raspberry Pi OS):
    sudo apt install -y python3-lgpio
Run:
    python3 pi/tools/gpio_scanner.py
Stop with Ctrl+C - it then prints a summary of how often each pin changed.
"""
import time

import lgpio

PINS = list(range(2, 28))  # BCM GPIO 2..27 (the 40-pin header)
POLL_S = 0.0005            # poll every 0.5 ms (~2 kHz)


def main():
    chip = lgpio.gpiochip_open(0)
    usable = []
    for pin in PINS:
        try:
            lgpio.gpio_claim_input(chip, pin)  # no pull-up/down -> does not change the circuit
            usable.append(pin)
        except lgpio.error:
            print(f"GPIO{pin}: busy (used by another driver), skipped")

    last = {p: lgpio.gpio_read(chip, p) for p in usable}
    counts = {p: 0 for p in usable}

    print("\nStart levels:", "  ".join(f"GPIO{p}={v}" for p, v in last.items()))
    print("\nWatching... spin a wheel or move something in front of an IR sensor. Ctrl+C to stop.\n")

    t0 = time.time()
    try:
        while True:
            for p in usable:
                v = lgpio.gpio_read(chip, p)
                if v != last[p]:
                    counts[p] += 1
                    last[p] = v
                    # print only the first few changes per pin so fast encoders don't flood the screen
                    if counts[p] <= 5 or counts[p] % 50 == 0:
                        print(f"{time.time() - t0:7.2f}s  GPIO{p:<2} -> {v}   (changes: {counts[p]})")
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n=== Summary: pins that changed ===")
        changed = {p: c for p, c in counts.items() if c > 0}
        if not changed:
            print("No pin changed.")
        for p, c in sorted(changed.items(), key=lambda x: -x[1]):
            print(f"GPIO{p:<2}: {c} changes")
        for p in usable:
            lgpio.gpio_free(chip, p)
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
