"""Physical parameters of the Sentinel-Pi robot (ET1 chassis, BO motors)."""
import math

WHEEL_DIAMETER_M = 0.065      # BO motor wheel, ~6.5 cm
COUNTS_PER_REV = 42.3         # measured: 423 counts / 10 turns (both edges counted).
                              # If the encoder disc has 20 slots, use 40.
TRACK_WIDTH_M = None          # TODO: distance between the two wheel centres (needed in Phase 5)

WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
METRES_PER_COUNT = WHEEL_CIRCUMFERENCE_M / COUNTS_PER_REV


def counts_to_m(counts):
    return counts * METRES_PER_COUNT
