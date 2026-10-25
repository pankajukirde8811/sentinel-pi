# Sentinel-Pi

Raspberry Pi 3B robot car built on the ET1 chassis, developed step by step
into an autonomous, sensor-fusion robot.

## Hardware
- Raspberry Pi 3B
- 2x DC motors with motor driver
- 2x wheel encoders
- HC-SR04 ultrasonic sensor
- 2x IR sensors
- Pi Camera (OV5647)

## Status
- [x] Remote access via SSH
- [x] Camera detected
- [x] Encoder pins found by GPIO scan
- [x] Ultrasonic verified
- [x] Motors verified
- [x] IR sensors identified
- [x] Phase 1: live camera stream Pi -> laptop (~32 fps, 640x480, ~20 kB/frame JPEG)
- [ ] Phase 2: I/O server (50 Hz telemetry, watchdog, obstacle stop) + laptop teleop

## Pin map (BCM)
| Function | GPIO |
|---|---|
| Motor left (PWM / IN1 / IN2) | 18 / 17 / 22 |
| Motor right (PWM / IN1 / IN2) | 19 / 24 / 4 |
| Encoder left / right (single channel) | 16 / 23 |
| Ultrasonic HC-SR04 trigger / echo | 25 / 27 |
| IR left / right | 20 / 21 |
| LED / button | 7 / 5 |

## Architecture
The Pi does fast I/O and safety; the laptop does the heavy computing.
- `pi/camera_server.py`: JPEG stream over ZeroMQ (port 5555)
- `pi/io_server.py`: 50 Hz sensor telemetry (port 5556), motor commands (port 5557),
  watchdog (motors stop after 0.3 s without commands), obstacle stop (< 15 cm)
- `laptop/camera_viewer.py`, `laptop/teleop.py`: live view, keyboard driving, CSV logging

## Structure
- `config/pins.py`: central GPIO pin map
- `robot/drive.py`: differential drive
- `tests/`: hardware test scripts
