import json
import tempfile
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from robot_explorer.configuration import parameters
from robot_explorer.core import locked_steering_profile


def setup(context):
    share=Path(get_package_share_directory('robot_explorer'))
    calibration=Path(LaunchConfiguration('calibration').perform(context)).expanduser().resolve()
    cfg=locked_steering_profile(json.loads(calibration.read_text()))
    bt=str(share/'behavior_trees/committed_path.xml')
    motion=LaunchConfiguration('allow_motion').perform(context).lower()=='true'
    ekf,slam,nav=parameters(cfg,bt, require_verified=motion)
    temp=Path(tempfile.mkdtemp(prefix='robot_explorer_'))
    paths=[]
    for name,data in [('ekf',ekf),('slam',slam),('nav',nav)]:
        p=temp/(name+'.yaml');p.write_text(yaml.safe_dump(data));paths.append(str(p))
    motion=LaunchConfiguration('allow_motion').perform(context).lower()=='true'
    nodes=[Node(package='robot_explorer',executable='navigation_map',name='navigation_map',output='screen'),
        Node(package='robot_explorer',executable='motion_observer',name='motion_observer',
        parameters=[dict(calibration=str(calibration))],output='screen'),
        Node(package='robot_explorer',executable='vehicle_bridge',name='vehicle_bridge',output='screen',
        parameters=[dict(calibration=str(calibration),allow_motion=motion)]),
        Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[paths[0]],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox'))/'launch/online_async_launch.py')),
            launch_arguments=dict(slam_params_file=paths[1],use_sim_time='false').items())]
    nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name='laser_mount',arguments=[
        '--x',str(cfg['laser_x_m']),'--y',str(cfg['laser_y_m']),'--z',str(cfg['laser_z_m']),
        '--yaw',str(cfg['laser_yaw_rad']),'--frame-id','base_link','--child-frame-id',cfg['laser_frame']]))
    nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name='imu_mount',arguments=[
        '--frame-id','base_link','--child-frame-id','imu_link']))
    names=['planner_server','controller_server','behavior_server','bt_navigator']
    for name in names:
        package={'planner_server':'nav2_planner','controller_server':'nav2_controller',
                 'behavior_server':'nav2_behaviors','bt_navigator':'nav2_bt_navigator'}[name]
        nodes.append(Node(package=package,executable=name,name=name,parameters=[paths[2]],output='screen'))
    nodes.extend([
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
            parameters=[dict(autostart=True,node_names=names,bond_timeout=4.)],output='screen'),
        Node(package='robot_explorer',executable='frontier_explorer',parameters=[dict(behavior_tree=bt,
            clearance=.216+cfg['clearance_margin_m'],retrace_speed=cfg['measured_speed_mps'],
            wheelbase=cfg['wheelbase_m'],max_steer=cfg['max_steer_rad'])],output='screen'),
        Node(package='robot_explorer',executable='exploration_dashboard',output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/cameras.launch.py')),
            launch_arguments={name:LaunchConfiguration(name) for role in ('front','rear')
                for name in ('start_'+role+'_camera',role+'_camera_device',role+'_camera_width',
                             role+'_camera_height',role+'_camera_fps',role+'_camera_pixel_format')}.items())])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration',default_value=str(Path.home()/'robot_nav_calibration.json')),
        DeclareLaunchArgument('allow_motion',default_value='false'),
        DeclareLaunchArgument('start_front_camera',default_value='false'),
        DeclareLaunchArgument('front_camera_device',default_value='/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0'),
        DeclareLaunchArgument('front_camera_width',default_value='640'),
        DeclareLaunchArgument('front_camera_height',default_value='480'),
        DeclareLaunchArgument('front_camera_fps',default_value='15.0'),
        DeclareLaunchArgument('front_camera_pixel_format',default_value='yuyv'),
        DeclareLaunchArgument('start_rear_camera',default_value='false'),
        DeclareLaunchArgument('rear_camera_device',default_value='/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:2:1.0-video-index0'),
        DeclareLaunchArgument('rear_camera_width',default_value='640'),
        DeclareLaunchArgument('rear_camera_height',default_value='480'),
        DeclareLaunchArgument('rear_camera_fps',default_value='15.0'),
        DeclareLaunchArgument('rear_camera_pixel_format',default_value='yuyv'),
        OpaqueFunction(function=setup)])
