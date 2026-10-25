"""GPIO pin map (BCM numbering) for the ET1 chassis."""
MOTOR_RIGHT = {"pwm": 19, "in1": 24, "in2": 4}    # Motor A
MOTOR_LEFT  = {"pwm": 18, "in1": 17, "in2": 22}   # Motor B
ULTRASONIC = {"trigger": 25, "echo": 27}
ENCODERS = {"left": 16, "right": 23}   # from gpio_scanner, 2026-10-25
LED = 7
BUTTON = 5
IR = {"left": 20, "right": 21}
