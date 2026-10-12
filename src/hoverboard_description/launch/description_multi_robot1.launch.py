#!/usr/bin/env python3
# Kept for the multi-robot launches: the same URDF as robot1 (frames robot1/...).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hoverboard_description'),
                'launch', 'description_launch.py')),
            launch_arguments={'robot_name': 'robot1'}.items()),
    ])
