# Dual Servo Motor Control

This demonstrates controlling two servo motors using FreeRTOS and ESP-IDF. The servos are driven using PWM signals, and smooth transitions between positions are implemented with timed increments.

The project includes control for two servos connected to GPIO pins, with smooth transitions between different positions for each servo.

## Supported Targets

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

## Example Overview

The example controls two servo motors using GPIO pins on an ESP32. The two servos are controlled by PWM signals, and smooth transitions between servo positions are achieved by gradually adjusting the pulse width of the PWM signal.

### Servo 1:

- Initial position at 90° (midpoint)
- Smooth transition from 0° to 180° over 100 ms and from 180° back to 0° over 2000 ms.

### Servo 2:

- Initially starts at 0°.
- Smooth transition from 0° to 180° over 1000 ms and back to 0° over 1000 ms.

## How to Use Example

Follow the detailed instructions provided for your specific Espressif chip:

- [ESP32 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [ESP32-S2 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
- [ESP32-C3 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)
- [ESP32-C6 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html)

## Example Folder Contents

The project **dual_servo_motor_control** contains the following files:
