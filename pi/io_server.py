#!/usr/bin/env python3
"""
sentinel-pi | pi/io_server.py   (runs on the Raspberry Pi)

50 times per second:
  1. read the newest command from the laptop                (ZeroMQ SUB, port 5557)
  2. read encoders, ultrasonic and IR sensors
  3. run the wheel speed controllers (in "speed" mode)
  4. apply SAFETY rules (on the Pi, never dependent on Wi-Fi):
       - watchdog: no command for 0.3 s -> motors stop
       - obstacle stop: forward motion blocked when ultrasonic < STOP_CM
  5. drive the motors, publish telemetry                    (ZeroMQ PUB, port 5556)

Command message (JSON), sent by the laptop at ~20 Hz:
    {"mode": "duty",  "left": -1..1,   "right": -1..1}     raw PWM duty (Phase 2)
    {"mode": "speed", "left": m/s,     "right": m/s}       closed-loop speed (Phase 3)
    optional: "kp", "ki" (controller gains), "accel" (m/s^2 target ramp)

Run from the repo root on the Pi:
    cd ~/sentinel-pi
    python3 -m pi.io_server
"""
import json
import signal
import threading
import time
from collections import deque

import lgpio
import zmq
from gpiozero import DigitalInputDevice, DistanceSensor

from config.pins import ENCODERS, IR, ULTRASONIC
from config.robot import BREAKAWAY_DUTY, METRES_PER_COUNT, MOTOR_MODEL, V_MAX, V_MIN
from robot.drive import Drive

RATE_HZ = 50
TELEMETRY_PORT = 5556
COMMAND_PORT = 5557
WATCHDOG_S = 0.3
STOP_CM = 15.0

# encoder / speed estimation
GLITCH_NS = 2_000_000      # edges closer than 2 ms are noise (max real rate ~12 ms/edge)
EDGE_WINDOW = 5            # average speed over the last 4 edge intervals (= 2 slots)
STOP_TIMEOUT_S = 0.15      # no edge for this long -> wheel is stopped

# speed controller defaults (tuned from the step response: tau ~0.1 s, dead time ~0.05 s)
KP = 0.8                   # duty per (m/s) error
KI = 6.0                   # duty per (m/s * s)
I_MAX = 0.3                # integrator limit (anti-windup)
ACCEL = 0.8                # m/s^2 target ramp


class Encoder:
    """Single-channel encoder read with lgpio callbacks.
    Uses the kernel edge timestamps, so speed comes from the TIME BETWEEN EDGES,
    which is much smoother than counting edges per 20 ms loop."""

    def __init__(self, chip, pin):
        self.chip, self.pin = chip, pin
        self._lock = threading.Lock()
        self.count = 0
        self.glitches = 0
        self._ticks = deque(maxlen=EDGE_WINDOW)
        self._last_tick = None
        self._last_edge_mono = 0
        lgpio.gpio_claim_alert(chip, pin, lgpio.BOTH_EDGES)
        self._cb = lgpio.callback(chip, pin, lgpio.BOTH_EDGES, self._edge)

    def _edge(self, chip, gpio, level, tick):
        if level > 1:                       # lgpio watchdog event, not an edge
            return
        with self._lock:
            if self._last_tick is not None:
                gap = tick - self._last_tick
                if gap < GLITCH_NS:         # electrical noise on the edge
                    self.glitches += 1
                    return
                if gap > STOP_TIMEOUT_S * 1e9:
                    self._ticks.clear()     # wheel was stopped: old edges are useless
            self._last_tick = tick
            self._ticks.append(tick)
            self._last_edge_mono = time.monotonic_ns()
            self.count += 1

    def speed(self):
        """Speed magnitude in m/s and the total edge count."""
        with self._lock:
            ticks = list(self._ticks)
            last_mono = self._last_edge_mono
            count = self.count
        if len(ticks) < 2:
            return 0.0, count
        age = (time.monotonic_ns() - last_mono) / 1e9
        if age > STOP_TIMEOUT_S:
            return 0.0, count
        n = len(ticks) - 1
        span = (ticks[-1] - ticks[0]) / 1e9
        v = n * METRES_PER_COUNT / span if span > 0 else 0.0
        # no new edge for longer than usual -> wheel is slowing down
        if age > 1.5 * span / n:
            v = min(v, METRES_PER_COUNT / age)
        return v, count

    def close(self):
        self._cb.cancel()
        lgpio.gpio_free(self.chip, self.pin)


class WheelController:
    """Feedforward (motor model) + PI with anti-windup and breakaway kick."""

    def __init__(self, model, kp=KP, ki=KI):
        self.k, self.b = model["k"], model["b"]
        self.kp, self.ki = kp, ki
        self.i = 0.0
        self.sign = 0

    def reset(self):
        self.i = 0.0
        self.sign = 0

    def update(self, target, meas, dt):
        """target: signed m/s, meas: speed magnitude m/s -> signed duty."""
        if abs(target) < V_MIN:
            self.reset()
            return 0.0
        sign = 1 if target > 0 else -1
        if sign != self.sign:               # direction change: start fresh
            self.reset()
            self.sign = sign
        t = min(abs(target), V_MAX)
        ff = (t - self.b) / self.k
        if meas < 0.02:
            # standstill: the speed reading needs 2 encoder edges (~50-80 ms), so the
            # error is meaningless here. Use feedforward + breakaway only, no P/I
            # (avoids the integrator winding up and the start-up overshoot).
            return sign * max(0.0, min(1.0, max(ff + self.i, BREAKAWAY_DUTY)))
        e = t - meas
        u = ff + self.kp * e + self.i
        saturated_high = u >= 1.0 and e > 0
        saturated_low = u <= 0.0 and e < 0
        if not (saturated_high or saturated_low):
            self.i = max(-I_MAX, min(I_MAX, self.i + self.ki * e * dt))
        return sign * max(0.0, min(1.0, u))


def ramp(current, goal, step):
    return current + max(-step, min(step, goal - current))


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
    chip = lgpio.gpiochip_open(0)
    enc_l = Encoder(chip, ENCODERS["left"])
    enc_r = Encoder(chip, ENCODERS["right"])
    ir_l = DigitalInputDevice(IR["left"], pull_up=True)    # value 1 = detected
    ir_r = DigitalInputDevice(IR["right"], pull_up=True)
    sonar = DistanceSensor(echo=ULTRASONIC["echo"], trigger=ULTRASONIC["trigger"],
                           max_distance=2, queue_len=3)
    drive = Drive()
    pid_l = WheelController(MOTOR_MODEL["left"])
    pid_r = WheelController(MOTOR_MODEL["right"])

    stop_event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    # ---------- state ----------
    period = 1.0 / RATE_HZ
    mode = "duty"
    cmd_l = cmd_r = 0.0          # newest command (duty or m/s)
    tgt_l = tgt_r = 0.0          # ramped speed targets
    accel = ACCEL
    last_cmd_t = -1e9
    out_prev = (None, None)
    sign_l = sign_r = 1          # encoders have no direction -> use commanded direction
    pos_l = pos_r = 0
    prev_cl = prev_cr = 0
    seq = 0
    worst_ms, n_win = 0.0, 0
    wd_prev, estop_prev = False, False
    n_wd = n_estop = 0

    print(f"[io_server] {RATE_HZ} Hz | telemetry :{TELEMETRY_PORT} | commands :{COMMAND_PORT} | "
          f"watchdog {WATCHDOG_S}s | stop < {STOP_CM} cm | Ctrl+C to stop")

    next_t = time.monotonic()
    prev_loop = next_t
    try:
        while not stop_event.is_set():
            now = time.monotonic()
            dt = now - prev_loop
            prev_loop = now

            # 1. newest command
            try:
                msg = json.loads(sub.recv(zmq.NOBLOCK))
                new_mode = msg.get("mode", "duty")
                if new_mode != mode:
                    mode = new_mode
                    pid_l.reset(); pid_r.reset()
                    tgt_l = tgt_r = 0.0
                cmd_l = float(msg.get("left", 0.0))
                cmd_r = float(msg.get("right", 0.0))
                if "kp" in msg:
                    pid_l.kp = pid_r.kp = float(msg["kp"])
                if "ki" in msg:
                    pid_l.ki = pid_r.ki = float(msg["ki"])
                if "accel" in msg:
                    accel = max(0.1, float(msg["accel"]))
                last_cmd_t = now
            except zmq.Again:
                pass
            except (ValueError, TypeError):
                pass

            # 2. sensors
            v_l, cl = enc_l.speed()
            v_r, cr = enc_r.speed()
            pos_l += sign_l * (cl - prev_cl)
            pos_r += sign_r * (cr - prev_cr)
            prev_cl, prev_cr = cl, cr
            dist_cm = sonar.distance * 100.0
            irl, irr = int(ir_l.value), int(ir_r.value)

            # 3. control
            if mode == "speed":
                tgt_l = ramp(tgt_l, cmd_l, accel * dt)
                tgt_r = ramp(tgt_r, cmd_r, accel * dt)
                out_l = pid_l.update(tgt_l, v_l, dt)
                out_r = pid_r.update(tgt_r, v_r, dt)
                forward = (tgt_l + tgt_r) > 0
            else:
                out_l = max(-1.0, min(1.0, cmd_l))
                out_r = max(-1.0, min(1.0, cmd_r))
                forward = (out_l + out_r) > 0

            # 4. safety (always last, always wins)
            wd_age = now - last_cmd_t
            wd_ok = wd_age < WATCHDOG_S
            estop = wd_ok and dist_cm < STOP_CM and forward
            if wd_ok != wd_prev or estop != estop_prev:      # log every safety event
                if not wd_ok and seq > 0 and last_cmd_t > 0:
                    print(f"[io_server] SAFETY: WATCHDOG stop (no command for {wd_age*1000:.0f} ms)")
                    n_wd += 1
                elif wd_ok and not wd_prev and n_wd:
                    print("[io_server] SAFETY: commands back, watchdog cleared")
                if estop and not estop_prev:
                    print(f"[io_server] SAFETY: OBSTACLE stop at {dist_cm:.1f} cm")
                    n_estop += 1
                elif estop_prev and not estop:
                    print("[io_server] SAFETY: obstacle cleared")
                wd_prev, estop_prev = wd_ok, estop
            if not wd_ok or estop:
                out_l = out_r = 0.0
                tgt_l = tgt_r = 0.0
                pid_l.reset(); pid_r.reset()

            # 5. outputs
            if (out_l, out_r) != out_prev:
                drive.set_speeds(out_l, out_r)
                out_prev = (out_l, out_r)
            if out_l:
                sign_l = 1 if out_l > 0 else -1
            if out_r:
                sign_r = 1 if out_r > 0 else -1

            pub.send_string(json.dumps({
                "seq": seq, "t": time.time(), "loop_ms": round(dt * 1000.0, 3), "mode": mode,
                "enc_l": cl, "enc_r": cr, "pos_l": pos_l, "pos_r": pos_r,
                "v_l": round(sign_l * v_l, 4), "v_r": round(sign_r * v_r, 4),
                "tgt_l": round(tgt_l, 4), "tgt_r": round(tgt_r, 4),
                "cmd_l": cmd_l, "cmd_r": cmd_r,
                "out_l": round(out_l, 4), "out_r": round(out_r, 4),
                "dist_cm": round(dist_cm, 1), "ir_l": irl, "ir_r": irr,
                "wd_ok": wd_ok, "estop": estop, "wd_age_ms": round(min(wd_age, 9.999) * 1000.0, 1),
                "n_wd": n_wd, "n_estop": n_estop,
                "glitch_l": enc_l.glitches, "glitch_r": enc_r.glitches,
                "kp": pid_l.kp, "ki": pid_l.ki,
            }))
            seq += 1

            worst_ms = max(worst_ms, dt * 1000.0)
            n_win += 1
            if n_win >= 2 * RATE_HZ:
                print(f"[io_server] {mode:5s} | worst loop {worst_ms:5.1f} ms | "
                      f"v L {v_l:.2f} R {v_r:.2f} m/s | dist {dist_cm:5.1f} cm | "
                      f"glitches {enc_l.glitches}/{enc_r.glitches} | "
                      f"{'LINK OK' if wd_ok else 'NO COMMANDS -> STOP'}"
                      f"{' | OBSTACLE STOP' if estop else ''}")
                worst_ms, n_win = 0.0, 0

            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[io_server] stopping motors")
        drive.close()
        for dev in (enc_l, enc_r, ir_l, ir_r, sonar):
            dev.close()
        lgpio.gpiochip_close(chip)
        pub.close(0)
        sub.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
