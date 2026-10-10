from gpiozero import OutputDevice
from time import sleep

channels = {
    "Motor A": {"en": 19, "in1": 24, "in2": 4},
    "Motor B": {"en": 18, "in1": 17, "in2": 22},
}

for name, p in channels.items():
    en, in1, in2 = OutputDevice(p["en"]), OutputDevice(p["in1"]), OutputDevice(p["in2"])
    print(f"{name}: FORWARD at full power for 3 s")
    en.on(); in1.on(); in2.off(); sleep(3)
    print(f"{name}: BACKWARD at full power for 3 s")
    in1.off(); in2.on(); sleep(3)
    en.off(); in1.off(); in2.off()
    en.close(); in1.close(); in2.close()
    print(f"{name}: stopped\n"); sleep(1)
