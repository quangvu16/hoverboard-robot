#!/usr/bin/env python3
# RViz with the navigation view (map, scan, costmaps, paths, goal tools).
#   ros2 launch hoverboard_nav display_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz = os.path.join(get_package_share_directory('robot_navigation'), 'rviz', 'robot_navigation2.rviz')
    return LaunchDescription([Node(package='rviz2', executable='rviz2', arguments=['-d', rviz], output='screen')])
