#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
if [ -f "$HOME/camera_ws/install/setup.bash" ]; then
  source "$HOME/camera_ws/install/setup.bash"
fi
sudo apt-get update
sudo apt-get install -y ros-lyrical-navigation2 ros-lyrical-nav2-bringup ros-lyrical-slam-toolbox ros-lyrical-robot-localization ros-lyrical-cv-bridge v4l-utils python3-colcon-common-extensions python3-serial python3-yaml python3-opencv python3-numpy
# This Pi already uses a source-built usb_cam in camera_ws. Lyrical's apt
# repository may not provide ros-lyrical-usb-cam, so prefer the working driver.
if ros2 pkg prefix usb_cam >/dev/null 2>&1; then
  echo 'Using existing usb_cam driver from the sourced ROS workspaces.'
elif apt-cache show ros-lyrical-usb-cam >/dev/null 2>&1; then
  sudo apt-get install -y ros-lyrical-usb-cam
else
  echo 'usb_cam is not available in the sourced workspaces or apt repository.' >&2
  echo 'Source the workspace used by the working front camera, then rerun this installer.' >&2
  exit 1
fi
mkdir -p "$HOME/nav_ws/src/robot_explorer"
SOURCE_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ "$SOURCE_DIR" != "$HOME/nav_ws/src/robot_explorer" ]; then
  if [ -f "$HOME/nav_ws/src/robot_explorer/package.xml" ]; then
    mkdir -p "$HOME/robot_backups/nav_ws"
    BACKUP_DIR="$HOME/robot_backups/nav_ws/robot_explorer_$(date +%Y%m%d_%H%M%S)_$$"
    cp -a "$HOME/nav_ws/src/robot_explorer" "$BACKUP_DIR"
    echo "Previous source saved to $BACKUP_DIR"
  fi
  cp -a "$SOURCE_DIR/." "$HOME/nav_ws/src/robot_explorer/"
fi
cd "$HOME/nav_ws"
# Restrict discovery to the intended source package, including when an older
# installer left a backup package under the workspace root.
colcon build --base-paths src/robot_explorer --symlink-install --packages-select robot_explorer
if [ ! -f "$HOME/robot_nav_calibration.json" ]; then
  cp "$HOME/nav_ws/src/robot_explorer/config/calibration.example.json" "$HOME/robot_nav_calibration.json"
fi
echo 'Installed. Existing calibration preserved; a newly copied template needs calibration. No robot motion was started.'
echo 'Next: source ~/nav_ws/install/setup.bash'
