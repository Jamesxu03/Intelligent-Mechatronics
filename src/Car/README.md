# Distributed Autonomous Mechatronics

This module utilizes a distributed architecture:
- **`host_pc_vision/`**: Runs OpenCV on a Linux/Mac environment to calculate trajectories and vision targets.
- **`embedded_mcu/`**: Runs on the PyBoard (MicroPython). These routines receive simulated telemetry from the PC and are responsible for the real-time PID balancing constraint equations.
