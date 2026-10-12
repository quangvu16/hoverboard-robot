#!/usr/bin/env python3
# Build a map with GMapping instead of Cartographer.
#   ros2 launch hoverboard_nav map_gmapping_launch.py
# Yahboom published a static base_link -> laser_frame transform here; the
# hoverboard URDF already provides base_link -> base_scan (the /scan frame),
# so nothing extra is needed.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_gmapping'), 'launch', 'slam_gmapping.launch.py'))),
    ])
