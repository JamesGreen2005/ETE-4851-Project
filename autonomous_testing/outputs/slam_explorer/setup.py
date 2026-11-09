from setuptools import setup
from glob import glob
setup(name='robot_explorer',version='0.17.0',packages=['robot_explorer'],
    data_files=[('share/ament_index/resource_index/packages',['resource/robot_explorer']),
                ('share/robot_explorer',['package.xml']),
                ('share/robot_explorer/launch',glob('launch/*.py')),
                ('share/robot_explorer/behavior_trees',glob('behavior_trees/*.xml'))],
    install_requires=['setuptools'],zip_safe=True,maintainer='Robot project',
    maintainer_email='robot@example.invalid',description='Four-wheel-steering SLAM and frontier exploration',license='Apache-2.0',
    entry_points={'console_scripts':[
        'vehicle_bridge=robot_explorer.bridge:main',
        'navigation_map=robot_explorer.navigation_map:main',
        'motion_observer=robot_explorer.motion_observer:main',
        'frontier_explorer=robot_explorer.explore:main',
        'exploration_dashboard=robot_explorer.dashboard:main']})
