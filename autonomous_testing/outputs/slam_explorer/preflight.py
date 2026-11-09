#!/usr/bin/env python3
"""Check Pi dependencies and calibration without opening serial or moving."""
import argparse
import importlib
import json
import sys
from pathlib import Path
from robot_explorer.core import require_calibration
from robot_explorer.configuration import parameters

def main():
    p=argparse.ArgumentParser();p.add_argument('--calibration',default=str(Path.home()/'robot_nav_calibration.json'))
    args=p.parse_args();errors=[]
    for module in ('numpy','rclpy','serial','yaml','cv2','cv_bridge','nav2_msgs.action','robot_localization','tf2_ros'):
        # robot_localization is a compiled package, not necessarily Python.
        if module=='robot_localization':continue
        try:importlib.import_module(module)
        except Exception as e:errors.append(module+': '+str(e))
    try:
        from ament_index_python.packages import get_package_share_directory
        for name in ('nav2_bringup','nav2_planner','nav2_controller','nav2_behaviors','nav2_bt_navigator',
                     'nav2_smac_planner','nav2_regulated_pure_pursuit_controller','robot_localization','slam_toolbox'):
            try:get_package_share_directory(name)
            except Exception:errors.append('ROS package missing: '+name)
    except Exception:errors.append('Source ROS and workspace first')
    try:
        c=require_calibration(json.loads(Path(args.calibration).expanduser().read_text()))
        parameters(c,'/tmp/tree.xml')
        from robot_explorer.motion_observer import camera_calibration
        for role in ('front','rear'):
            path=c.get(role+'_odometry_calibration')
            if path:camera_calibration(json.loads(Path(path).expanduser().read_text()))
    except Exception as e:errors.append('Calibration: '+str(e))
    if errors:
        print('\n'.join(errors));return 1
    print('Dependency/calibration checks passed. ROS launch and physical validation still required.')
    return 0

if __name__=='__main__':sys.exit(main())
