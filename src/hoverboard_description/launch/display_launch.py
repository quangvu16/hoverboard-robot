#!/usr/bin/env python3
# Look at the model in RViz, with sliders for the wheel joints.
#   ros2 launch hoverboard_description display_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hoverboard_description')
    with open(os.path.join(share, 'urdf', 'hoverboard.urdf')) as f:
        urdf = f.read()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'display.rviz')]),
    ])
