"""Differential drive control for the Sentinel-Pi robot."""
from gpiozero import Motor
from time import sleep
from config.pins import MOTOR_LEFT, MOTOR_RIGHT

# Set to True if that wheel turns the wrong way on forward()
INVERT_LEFT = False
INVERT_RIGHT = False


class Drive:
    def __init__(self, speed=0.6):
        self.speed = speed
        self.left = Motor(forward=MOTOR_LEFT["in1"], backward=MOTOR_LEFT["in2"],
                          enable=MOTOR_LEFT["pwm"])
        self.right = Motor(forward=MOTOR_RIGHT["in1"], backward=MOTOR_RIGHT["in2"],
                           enable=MOTOR_RIGHT["pwm"])

    def _set(self, motor, value, invert):
        value = -value if invert else value
        if value > 0:
            motor.forward(min(value, 1))
        elif value < 0:
            motor.backward(min(-value, 1))
        else:
            motor.stop()

    def set_speeds(self, left, right):
        self._set(self.left, left, INVERT_LEFT)
        self._set(self.right, right, INVERT_RIGHT)

    def forward(self, speed=None):
        s = speed or self.speed
        self.set_speeds(s, s)

    def backward(self, speed=None):
        s = speed or self.speed
        self.set_speeds(-s, -s)

    def left_turn(self, speed=None):
        s = speed or self.speed
        self.set_speeds(-s, s)

    def right_turn(self, speed=None):
        s = speed or self.speed
        self.set_speeds(s, -s)

    def stop(self):
        self.set_speeds(0, 0)

    def close(self):
        self.stop()
        self.left.close()
        self.right.close()


if __name__ == "__main__":
    d = Drive(speed=0.6)
    try:
        for name, action in [("forward", d.forward), ("backward", d.backward),
                             ("turn left", d.left_turn), ("turn right", d.right_turn)]:
            print(name)
            action()
            sleep(1.5)
            d.stop()
            sleep(0.7)
    finally:
        d.close()
        print("done")
