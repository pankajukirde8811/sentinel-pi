from gpiozero import LED, Button
from signal import pause

led = LED(7)
button = Button(5)   # pull-up: pressed = connected to GND

def pressed():
    print("Button pressed -> LED ON")
    led.on()

def released():
    print("Button released -> LED OFF")
    led.off()

button.when_pressed = pressed
button.when_released = released

print("Blinking LED 3 times...")
led.blink(on_time=0.3, off_time=0.3, n=3, background=False)
print("Now press the button. Ctrl+C to stop.")
pause()
