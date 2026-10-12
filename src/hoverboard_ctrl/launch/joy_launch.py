#!/usr/bin/env python3
# Gamepad driving: joy_node + hoverboard_joy, mapping in config/joy.yaml.
#   ros2 launch hoverboard_ctrl joy_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('hoverboard_ctrl'), 'config', 'joy.yaml')
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[params]),
        Node(package='hoverboard_ctrl', executable='hoverboard_joy', name='joy_ctrl',
             output='screen', parameters=[params]),
    ])
