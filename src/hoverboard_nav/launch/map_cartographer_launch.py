#!/usr/bin/env python3
# Build a map with Cartographer (no navigation). Drive with hoverboard_ctrl.
#   ros2 launch hoverboard_nav map_cartographer_launch.py
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>   (or save_map_launch.py)
# Uses robot_cartographer's config (robot_cartographer/config/robot_2d.lua),
# tuned for this robot - not Yahboom's lds_2d.lua. Needs robot_bringup
# (base:=hoverboard) running for /scan, /odom and the TF tree.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_cartographer'), 'launch',
                'cartographer_nav.launch.py')),
            launch_arguments={'nav': 'false', 'rviz': LaunchConfiguration('rviz')}.items()),
    ])
