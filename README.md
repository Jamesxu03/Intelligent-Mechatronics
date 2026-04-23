# Intelligent Mechatronics V2.3 🤖

A distributed robotics platform featuring real-time gesture recognition (MediaPipe) and autonomous vehicle perception (OpenCV), built with professional software engineering practices.

## Project Structure

```
src/
├── robotic_manipulator/           # Gesture recognition + sensor fusion
│   ├── hand_controller.py         # MediaPipe Vision + Arduino serial integration
│   └── rps_firmware.ino           # Arduino firmware for servo hand + flex sensors
└── autonomous_platform/
    ├── host_pc_vision/            # Runs on Host PC (Linux/Mac)
    │   ├── perception.py          # Lane detection + obstacle identification (OpenCV)
    │   └── trajectory_tracking.py # PID steering from visual offsets
    └── embedded_mcu/              # Runs on PyBoard (MicroPython)
        ├── main.py                # DIP switch menu entry point
        ├── challenge_5.py         # Self-balancing Segway (complementary filter + PID)
        ├── pid.py                 # Encapsulated PID controller
        ├── config.py              # Hardware pin assignments
        ├── motor_driver.py        # TB6612 dual motor driver
        ├── mpu6050_driver.py      # IMU driver with gyro calibration
        └── oled_938.py            # SSD1306 OLED display driver
tests/
└── test_system.py                 # Offline CI/CD-friendly unit tests
```

## Architectural Highlights

- **OOP Architecture**: Classes map to physical domains (`VisualGestureRecognizer`, `AutonomousPerceptionModule`, `PIDController`)
- **Robust Error Handling**: `logging` module throughout; `try-except` with `raise` on fatal errors, graceful degradation for optional hardware
- **CI/CD Pipeline**: GitHub Actions runs `pytest` automatically on every push and PR
- **Distributed Design**: Host PC vision code separated from embedded MicroPython firmware
- **Type Hinting**: `numpy` array types and `typing` generics on public methods

## Quick Start

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
2. Run the gesture recognition system:
   ```bash
   python src/robotic_manipulator/hand_controller.py --port /dev/cu.usbserial-0001
   ```
3. Run the autonomous perception pipeline:
   ```bash
   python src/autonomous_platform/host_pc_vision/perception.py
   ```

## Running Tests

```bash
pytest tests/
```

Tests run offline without hardware by using simulation mode for both the camera and MediaPipe modules.
