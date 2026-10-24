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
- [ ] IR sensors identified

## Structure
- `config/pins.py`: central GPIO pin map
- `tests/`: hardware test scripts
