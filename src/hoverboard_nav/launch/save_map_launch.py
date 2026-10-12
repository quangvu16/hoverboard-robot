#!/usr/bin/env python3
# Save the current /map to ~/maps/<map_name>.yaml/.pgm (Yahboom wrote into the
# source tree through a relative path).
#   ros2 launch hoverboard_nav save_map_launch.py map_name:=kitchen
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _saver(context):
    d = os.path.expanduser(LaunchConfiguration('maps_dir').perform(context))
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, LaunchConfiguration('map_name').perform(context))
    return [Node(package='nav2_map_server', executable='map_saver_cli', output='screen',
                 arguments=['-f', path], parameters=[{'save_map_timeout': 10000.0}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='hoverboard_map'),
        DeclareLaunchArgument('maps_dir', default_value='~/maps'),
        OpaqueFunction(function=_saver),
    ])
