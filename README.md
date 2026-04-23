# Intelligent Mechatronics — Distributed Robotics Platform 🤖

[![CI/CD](https://github.com/Jamesxu03/Intelligent-Mechatronics/actions/workflows/python-tests.yml/badge.svg)](https://github.com/Jamesxu03/Intelligent-Mechatronics/actions)
![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.8%2B-green)
![MediaPipe](https://img.shields.io/badge/MediaPipe-0.10%2B-orange)
![License](https://img.shields.io/badge/License-Academic-lightgrey)

A production-grade robotics codebase integrating **real-time computer vision** (OpenCV, MediaPipe) with **embedded PID control** (MicroPython) across two hardware platforms: a gesture-controlled robotic hand and an autonomous self-balancing vehicle.

---

## Architecture

The system uses a **distributed architecture** separating high-level perception (Host PC) from real-time control (Embedded MCU), connected via serial telemetry.

```
┌──────────────────────────────┐    Serial/BLE     ┌─────────────────────────────┐
│       HOST PC (Python)       │ ◄──────────────── │    EMBEDDED MCU (µPython)   │
│                              │                    │                             │
│  ┌─────────────────────┐     │                    │  ┌───────────────────────┐  │
│  │  perception.py      │     │                    │  │  challenge_5.py       │  │
│  │  - Canny Edge       │     │                    │  │  - Complementary      │  │
│  │  - Hough Transform  │────►│   offset_px        │  │    Filter @ 200Hz     │  │
│  │  - HSV Obstacle     │     │   ───────────────► │  │  - PID Controller     │  │
│  │    Detection        │     │                    │  │  - Motor Drive        │  │
│  └─────────────────────┘     │                    │  └───────────────────────┘  │
│                              │                    │                             │
│  ┌─────────────────────┐     │                    │  ┌───────────────────────┐  │
│  │  hand_controller.py │     │  PHYSICAL_MOVE:n   │  │  rps_firmware.ino     │  │
│  │  - MediaPipe Hands  │◄────│  ◄───────────────  │  │  - Servo Control      │  │
│  │  - Gesture Fusion   │     │                    │  │  - Flex Sensors        │  │
│  │  - Redundancy Check │     │                    │  │  - LED Feedback        │  │
│  └─────────────────────┘     │                    │  └───────────────────────┘  │
└──────────────────────────────┘                    └─────────────────────────────┘
```

---

## Project Structure

```
src/
├── robotic_manipulator/                 # Gesture Recognition Platform
│   ├── hand_controller.py               # MediaPipe Vision + dual-redundant sensor fusion
│   └── rps_firmware.ino                 # Arduino firmware: servos, flex sensors, serial comms
│
└── autonomous_platform/                 # Autonomous Self-Balancing Vehicle
    ├── host_pc_vision/                  # Runs on Host PC (Linux/Mac)
    │   ├── perception.py                # Lane detection (Hough) + obstacle ID (HSV contours)
    │   └── trajectory_tracking.py       # PID steering corrections from visual offsets
    │
    └── embedded_mcu/                    # Runs on PyBoard (MicroPython)
        ├── main.py                      # DIP-switch menu entry point
        ├── challenge_5.py               # Self-balancing control loop (complementary filter + PID)
        ├── pid.py                       # Encapsulated PID controller with anti-windup
        ├── config.py                    # Hardware pin assignments & calibration constants
        ├── motor_driver.py              # TB6612 dual H-bridge motor driver
        ├── mpu6050_driver.py            # MPU6050 IMU driver with gyro bias calibration
        ├── oled_938.py                  # SSD1306 128×64 OLED display driver
        ├── oled_display.py              # Compatibility wrapper
        └── i2c_bus.py                   # Shared I²C bus singleton

tests/
└── test_system.py                       # CI/CD-compatible offline unit tests
```

---

## Tech Stack

| Layer | Technology | Purpose |
|---|---|---|
| **Vision** | OpenCV 4.8+, MediaPipe 0.10+ | Lane detection, obstacle ID, hand landmark tracking |
| **Control** | Custom PID (Python) | Pitch stabilisation, steering corrections |
| **Sensing** | MPU6050 (I²C), Flex Sensors (ADC) | 6-axis IMU, physical gesture input |
| **Actuation** | TB6612 H-Bridge, Servos (PWM) | Dual motors, 5-finger robotic hand |
| **Comms** | PySerial, UART/BLE | Host PC ↔ MCU telemetry bridge |
| **Firmware** | MicroPython (PyBoard), Arduino C++ | Real-time embedded control |
| **CI/CD** | GitHub Actions, Pytest | Automated testing on every push |
| **Standards** | OOP, `logging`, `argparse`, type hints | Production-ready code quality |

---

## Quick Start

> **⚠️ Hardware Notice:** The vision modules require a webcam. The robotic hand module communicates with an Arduino over serial — pass your machine's serial port via `--port`. All hardware interfaces degrade gracefully in simulation mode; the test suite runs fully offline.

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run Gesture Recognition System
```bash
# Replace the --port value with YOUR system's serial port
python src/robotic_manipulator/hand_controller.py --port /dev/ttyUSB0
```
- Requires a webcam and Arduino with flex sensor PCB
- The `--port` flag accepts any serial device (Linux: `/dev/ttyUSB0`, Mac: `/dev/cu.usbserial-*`, Windows: `COM3`)
- Performs dual-redundant gesture validation (visual + physical)

### 3. Run Autonomous Perception Pipeline
```bash
python src/autonomous_platform/host_pc_vision/perception.py
```
- Requires a webcam; detects lanes and red obstacles in real-time
- Press `q` to quit the dashboard

### 4. Deploy to PyBoard (Self-Balancing Segway)
```bash
# Copy embedded_mcu/ contents to PyBoard flash
# Set DIP switch to 010, press USER button to start balancing
```

---

## Running Tests

```bash
pytest tests/
```

All tests run **offline without hardware** using simulation mode — cameras are mocked, serial interfaces are gracefully degraded. The CI/CD pipeline validates this on every push via GitHub Actions.

---

## Key Engineering Decisions

- **Dual-redundancy pattern**: Visual gesture predictions are cross-validated against physical flex sensor telemetry before any robot command is issued
- **Encapsulated PID**: Integral state lives inside `PIDController` with built-in anti-windup, eliminating caller-side state management bugs
- **Complementary filter @ 200Hz**: Fuses accelerometer (drift-free, noisy) with gyroscope (smooth, drifting) for stable pitch estimation
- **Graceful hardware degradation**: All hardware interfaces (`cv2.VideoCapture`, `serial.Serial`) fall back to simulation mode in CI/CD, so tests never require physical devices
