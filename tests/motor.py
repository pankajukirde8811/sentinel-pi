from gpiozero import Motor
from time import sleep

m = Motor(forward=24, backward=4, enable=19)

print("Forward, slow");   m.forward(0.4);  sleep(2)
print("Forward, fast");   m.forward(0.8);  sleep(2)
print("Stop");            m.stop();        sleep(1)
print("Backward, slow");  m.backward(0.4); sleep(2)
m.stop()
print("Done")
