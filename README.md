# Differential-Drive Autonomous Robot

This repository contains the hardware documentation, STM32 firmware, ROS 2 software, and CoppeliaSim model for our differential-drive autonomous robot.

## Primary Architecture

- NVIDIA Jetson Orin Nano
- Ubuntu 22.04 through JetPack 6.2.1
- ROS 2 Humble
- NUCLEO-F767ZI real-time controller
- Two encoder-equipped Pololu 37D gearmotors
- Dual-channel brushed DC motor driver
- RPLIDAR C1
- RGB camera
- IMU, power monitor, bumpers, and optional ToF sensors
- CoppeliaSim differential-drive simulation

## Repository Organization

- `firmware/` — NUCLEO-F767ZI firmware
- `ros2_ws/` — ROS 2 workspace and packages
- `simulation/` — CoppeliaSim models and configuration
- `hardware/` — Wiring, pinouts, datasheets, and mechanical files
- `docs/` — Project documentation
- `scripts/` — Setup and utility scripts
- `tests/` — Hardware and software testing records

## Initial Milestones

1. Read both motor encoders using the F767ZI.
2. Control both motors with PWM and direction signals.
3. Implement closed-loop wheel-speed control.
4. Establish Jetson-to-F767ZI UART communication.
5. Implement ROS 2 keyboard teleoperation.
6. Publish differential-drive odometry.
7. Build the corresponding CoppeliaSim model.
8. Integrate localization, SLAM, and Nav2.
