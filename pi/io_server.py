#!/usr/bin/env python3
"""
sentinel-pi | pi/io_server.py   (Phase 2: runs on the Raspberry Pi)

What it does, 50 times per second:
  1. reads the newest motor command from the laptop      (ZeroMQ SUB, port 5557)
  2. reads encoders, ultrasonic and IR sensors
  3. applies the SAFETY rules (on the Pi, never dependent on Wi-Fi):
       - watchdog: no command from the laptop for 0.3 s -> motors stop
       - obstacle stop: forward motion is blocked when ultrasonic < STOP_CM
  4. drives the motors and publishes telemetry           (ZeroMQ PUB, port 5556)

Run from the repo root on the Pi:
    cd ~/sentinel-pi
    python3 -m pi.io_server
"""
import json
import signal
import threading
import time

import zmq
from gpiozero import DigitalInputDevice, DistanceSensor

from config.pins import ENCODERS, IR, ULTRASONIC
from robot.drive import Drive

RATE_HZ = 50            # control / telemetry rate
TELEMETRY_PORT = 5556   # Pi -> laptop
COMMAND_PORT = 5557     # laptop -> Pi
WATCHDOG_S = 0.3        # stop motors if the laptop is silent this long
STOP_CM = 15.0          # block forward motion below this distance
MAX_DUTY = 1.0          # motor duty limit (0..1)


class Encoder:
    """Single-channel wheel encoder. Counts BOTH edges for double resolution."""

    def __init__(self, pin):
        self._dev = DigitalInputDevice(pin)
        self._lock = threading.Lock()
        self._count = 0
        self._dev.when_activated = self._tick
        self._dev.when_deactivated = self._tick

    def _tick(self):
        with self._lock:
            self._count += 1

    def read(self):
        with self._lock:
            return self._count

    def close(self):
        self._dev.close()


def clamp(x, limit=MAX_DUTY):
    return max(-limit, min(limit, x))


def main():
    # ---------- network ----------
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 50)
    pub.bind(f"tcp://*:{TELEMETRY_PORT}")

    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.CONFLATE, 1)          # keep only the newest command
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.bind(f"tcp://*:{COMMAND_PORT}")

    # ---------- hardware ----------
    enc_l = Encoder(ENCODERS["left"])
    enc_r = Encoder(ENCODERS["right"])
    ir_l = DigitalInputDevice(IR["left"], pull_up=True)    # value 1 = detected
    ir_r = DigitalInputDevice(IR["right"], pull_up=True)
    sonar = DistanceSensor(echo=ULTRASONIC["echo"], trigger=ULTRASONIC["trigger"],
                           max_distance=2, queue_len=3)
    drive = Drive()

    stop_event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    # ---------- state ----------
    period = 1.0 / RATE_HZ
    cmd_l = cmd_r = 0.0
    out_prev = (None, None)
    last_cmd_t = -1e9
    sign_l = sign_r = 1          # encoders have no direction -> use commanded direction
    pos_l = pos_r = 0            # signed counts
    prev_l, prev_r = enc_l.read(), enc_r.read()
    seq = 0
    worst_ms, n_win = 0.0, 0

    print(f"[io_server] running at {RATE_HZ} Hz | telemetry :{TELEMETRY_PORT} | "
          f"commands :{COMMAND_PORT} | watchdog {WATCHDOG_S}s | stop < {STOP_CM} cm")
    print("[io_server] Ctrl+C to stop")

    next_t = time.monotonic()
    prev_loop = next_t
    try:
        while not stop_event.is_set():
            now = time.monotonic()
            loop_ms = (now - prev_loop) * 1000.0
            prev_loop = now

            # 1. newest command
            try:
                msg = json.loads(sub.recv(zmq.NOBLOCK))
                cmd_l = clamp(float(msg.get("left", 0.0)))
                cmd_r = clamp(float(msg.get("right", 0.0)))
                last_cmd_t = now
            except zmq.Again:
                pass
            except (ValueError, TypeError):
                pass  # ignore malformed message

            # 2. sensors
            cl, cr = enc_l.read(), enc_r.read()
            dl, dr = cl - prev_l, cr - prev_r
            prev_l, prev_r = cl, cr
            pos_l += sign_l * dl
            pos_r += sign_r * dr
            dist_cm = sonar.distance * 100.0
            irl, irr = int(ir_l.value), int(ir_r.value)

            # 3. safety
            wd_ok = (now - last_cmd_t) < WATCHDOG_S
            out_l, out_r = (cmd_l, cmd_r) if wd_ok else (0.0, 0.0)
            estop = dist_cm < STOP_CM and (out_l + out_r) > 0   # moving forward into obstacle
            if estop:
                out_l = out_r = 0.0

            if (out_l, out_r) != out_prev:      # only touch PWM when something changed
                drive.set_speeds(out_l, out_r)
                out_prev = (out_l, out_r)
            if out_l:
                sign_l = 1 if out_l > 0 else -1
            if out_r:
                sign_r = 1 if out_r > 0 else -1

            # 4. telemetry
            pub.send_string(json.dumps({
                "seq": seq, "t": time.time(), "loop_ms": round(loop_ms, 3),
                "enc_l": cl, "enc_r": cr, "dl": dl, "dr": dr,
                "pos_l": pos_l, "pos_r": pos_r,
                "dist_cm": round(dist_cm, 1), "ir_l": irl, "ir_r": irr,
                "cmd_l": cmd_l, "cmd_r": cmd_r, "out_l": out_l, "out_r": out_r,
                "wd_ok": wd_ok, "estop": estop,
            }))
            seq += 1

            # console status every 2 s
            worst_ms = max(worst_ms, loop_ms)
            n_win += 1
            if n_win >= 2 * RATE_HZ:
                print(f"[io_server] worst loop {worst_ms:5.1f} ms | dist {dist_cm:5.1f} cm | "
                      f"enc L {cl} R {cr} | IR {irl}{irr} | "
                      f"{'LINK OK' if wd_ok else 'NO COMMANDS -> STOP'}"
                      f"{' | OBSTACLE STOP' if estop else ''}")
                worst_ms, n_win = 0.0, 0

            # 5. wait for next period
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()   # overrun: resync instead of bursting
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[io_server] stopping motors")
        drive.close()
        for dev in (enc_l, enc_r, ir_l, ir_r, sonar):
            dev.close()
        pub.close(0)
        sub.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
