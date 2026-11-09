"""USB front/rear drivers only; never open the MCU or start navigation."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    nodes = []
    selected = []
    for role in ('front', 'rear'):
        if value('start_'+role+'_camera').lower() != 'true':
            continue
        device = value(role+'_camera_device').strip()
        if not device or not Path(device).is_absolute() or not Path(device).exists():
            raise ValueError('Set '+role+'_camera_device to the verified capture device '
                             'from /dev/v4l/by-id/ or /dev/v4l/by-path/.')
        resolved = Path(device).resolve()
        if resolved in selected:
            raise ValueError('Front and rear cameras must use different capture devices')
        selected.append(resolved)
        nodes.append(Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
                     namespace='camera' if role == 'front' else 'camera/rear', output='screen',
                     parameters=[{
                         'video_device': device, 'image_width': int(value(role+'_camera_width')),
                         'image_height': int(value(role+'_camera_height')),
                         'framerate': float(value(role+'_camera_fps')),
                         'pixel_format': value(role+'_camera_pixel_format'), 'io_method': 'mmap',
                         'frame_id': 'camera_optical_frame' if role == 'front' else 'rear_camera_optical_frame',
                         'camera_name': 'nexigo_n60' if role == 'front' else 'rear_camera'}]))
    return nodes


def generate_launch_description():
    arguments = []
    for role, port in [('front', '0'), ('rear', '1')]:
        defaults = {
            'start_'+role+'_camera': 'false' if role == 'front' else 'true',
            role+'_camera_device': '/dev/v4l/by-path/platform-xhci-hcd.'+port+'-usb-0:2:1.0-video-index0',
            role+'_camera_width': '640', role+'_camera_height': '480',
            role+'_camera_fps': '15.0', role+'_camera_pixel_format': 'yuyv'}
        arguments.extend(DeclareLaunchArgument(name, default_value=default)
                         for name, default in defaults.items())
    return LaunchDescription(arguments+[OpaqueFunction(function=setup)])
