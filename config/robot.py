"""Physical parameters of the Sentinel-Pi robot (ET1 chassis, BO motors)."""
import math

WHEEL_DIAMETER_M = 0.065      # BO motor wheel, ~6.5 cm
COUNTS_PER_REV = 42.3         # measured: 423 counts / 10 turns (both edges counted).
                              # re-measure after the encoder glitch filter (Phase 3)
TRACK_WIDTH_M = 0.10          # wheel centre to wheel centre (measured ~10 cm, calibrate in Phase 5)

WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
METRES_PER_COUNT = WHEEL_CIRCUMFERENCE_M / COUNTS_PER_REV

# Motor model from 3 duty sweeps (wheels in air, duty >= 0.5):  v = K * duty + B
# used as FEEDFORWARD by the speed controller: duty = (v - B) / K
MOTOR_MODEL = {
    "left":  {"k": 0.324, "b": 0.075},
    "right": {"k": 0.314, "b": 0.086},
}
BREAKAWAY_DUTY = 0.5          # duty needed to start a wheel from standstill (deadband 0.4-0.5)
V_MIN = 0.18                  # m/s: slower targets are treated as stop (wheel cannot run steadily)
V_MAX = 0.38                  # m/s: leave headroom below the ~0.40 m/s maximum


def counts_to_m(counts):
    return counts * METRES_PER_COUNT
