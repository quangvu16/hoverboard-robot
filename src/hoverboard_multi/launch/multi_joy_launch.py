#!/usr/bin/env python3
# One gamepad drives all listed robots with the same command.
#   ros2 launch hoverboard_multi multi_joy_launch.py robots:=robot1,robot2
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params = os.path.join(get_package_share_directory('hoverboard_ctrl'), 'config', 'joy.yaml')
    topics = [f'/{r}/cmd_vel' for r in LaunchConfiguration('robots').perform(context).split(',') if r]
    return [
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[params]),
        Node(package='hoverboard_ctrl', executable='hoverboard_joy', name='joy_ctrl', output='screen',
             parameters=[params, {'cmd_vel_topics': topics}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='robot1,robot2'),
        OpaqueFunction(function=_nodes),
    ])
