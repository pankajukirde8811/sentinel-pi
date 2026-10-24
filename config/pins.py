"""GPIO pin map (BCM numbering) for the ET1 chassis."""
MOTOR_RIGHT = {"pwm": 19, "in1": 24, "in2": 4}    # Motor A
MOTOR_LEFT  = {"pwm": 18, "in1": 17, "in2": 22}   # Motor B
ULTRASONIC = {"trigger": 25, "echo": 27}
ENCODERS = {"a": 23, "b": 16}   # to be confirmed (left/right)
LED = 7
BUTTON = 5
IR = {}                          # not identified yet
