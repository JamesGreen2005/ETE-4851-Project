# Startup commands

Use the actual Pi address if it has changed. The last-known address was 10.110.135.247. Do not launch duplicate sensor drivers. Stop an older navigation launch before updating.

## Install from the repository checkout

In Windows PowerShell, from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File .\autonomous_testing\outputs\slam_explorer\Update-Autonomy.ps1 -PiHost 10.110.135.247
```

The installer preserves the existing Pi calibration. It does not flash the Nucleo or start movement. It needs Pi internet/sudo for dependencies.

## Pi terminal 1 LiDAR

```bash
source /opt/ros/lyrical/setup.bash
source ~/lidar_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=460800
```

## Pi terminal 2 Both cameras

```bash
source /opt/ros/lyrical/setup.bash
source ~/camera_ws/install/setup.bash
source ~/nav_ws/install/setup.bash
ros2 launch robot_explorer cameras.launch.py start_front_camera:=true
```

## Windows tunnel and Pi terminal 3

In Windows PowerShell:

```powershell
ssh -L 8005:127.0.0.1:8005 swerve@10.110.135.247
```

In that connected Pi terminal, begin with motion disabled:

```bash
source /opt/ros/lyrical/setup.bash
source ~/camera_ws/install/setup.bash
source ~/nav_ws/install/setup.bash
python3 ~/slam_explorer_autonomy_update/preflight.py --calibration ~/robot_nav_calibration.json
ros2 launch robot_explorer explore.launch.py calibration:=$HOME/robot_nav_calibration.json allow_motion:=false
```

Open the complete new token URL printed by the launch. After stationary checks, Ctrl+C stops that launch. In the same sourced Pi terminal, the supervised motion-capable launch is:

```bash
ros2 launch robot_explorer explore.launch.py calibration:=$HOME/robot_nav_calibration.json allow_motion:=true
```

Open its new token URL, wait for navigation services, then explicitly click Start exploration. Startup itself does not arm or move the robot. Reset navigation stops/cancels the session and waits for Ready; press Start separately afterward. Actual missing clearance or sensor faults can still prevent movement.
