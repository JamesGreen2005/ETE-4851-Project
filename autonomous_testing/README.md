# Autonomous testing files

This folder preserves the Raspberry Pi dashboard and ROS 2 autonomy testing files, the matching FW7 Nucleo reference, tests, calibration snapshot and earlier dashboard release archives. The latest source is SLAM Explorer 0.17; it is not a claim of validated uninterrupted grass navigation.

## Start here

- [File guide](FILE_GUIDE.md): what every included file does.
- [Startup commands](START_COMMANDS.md): installation, LiDAR, both cameras, stationary checks and supervised motion launch.
- [Validation record](VALIDATION.md): what passed and what remained unresolved.
- `outputs/slam_explorer/`: current readable source, configuration and tests.
- `outputs/slam_explorer_pi.zip`: current 0.17 Pi installation archive, kept beside the source so Update-Autonomy.ps1 can find it.
- `releases/`: earlier dashboard/navigation archives, each containing its own version of the source. These are historical alternatives, not simultaneous stacks.
- `firmware/fw7/`: matching encoder-diagnostic firmware source reference, compiled binary and uploader. No firmware flash is needed for dashboard-only updates.
- `firmware/motor_imu/` and `firmware/pi_motor_imu/`: original MATLAB model inputs and CubeMX project/core reference.
- `calibration/`: clearly labeled last-known robot calibration, not a fresh device download.
- `diagnostics/`: read-only or motion-disabled checks used during testing.

## What the software actually uses

The Pi runs SLAM Toolbox, robot_localization EKF, Nav2 Reeds-Shepp planning, the explorer, the USB serial vehicle bridge and the dashboard. The Pi speed controller sends signed duty and steering commands; the Nucleo enforces the command protocol and watchdog and reports encoder/IMU/current telemetry.

IMU heading and calibrated encoder speed feed the EKF. LiDAR provides scan odometry, mapping and obstacle checks. Motor current adjusts wheel confidence; it is not a position measurement. Front/rear video is available. Optional calibrated camera ground-motion odometry is implemented but disabled until calibration is supplied. Camera obstacle-clearance detection is not implemented.

Rear coverage uses the user-selected one-of-twenty sector threshold. Missing sectors remain unknown. The 100 percent motor setting is a maximum duty ceiling; the cruise target remains the measured speed. One serial owner and one navigation/dashboard stack must run at a time.

## Firmware build scope

`firmware/fw7/Vehicle_Drive.bin` and the patched `Vehicle_Drive_generated.c` preserve the diagnostic revision associated with PC10/PC11 and PC8/PC9 encoders. The MATLAB scripts are historical model-generation inputs; regenerating the model alone is not guaranteed to reproduce the patched FW7 binary. MATLAB/Simulink, STM32 support packages, the generated support headers and vendor toolchains are external dependencies. No claim of a clean standalone firmware rebuild is made for this archive.

## Local checks

From `outputs/slam_explorer`, use Python with numpy, opencv-python and PyYAML installed:

```sh
python -m unittest discover -s tests
node tests/test_dashboard.cjs
```

ROS runtime packages, the source-built usb_cam camera driver, sllidar_ros2 and the actual on-device calibration remain installation dependencies; they are not vendored toolchains in this archive.
