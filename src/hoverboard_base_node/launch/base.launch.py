#!/usr/bin/env python3
# The hoverboard base on its own: the UDP bridge to the ESP32 (/odom, the
# odom->base_footprint TF, /wheel_joint_states, /cmd_vel, /Buzzer, the
# ultrasonic topics), configured from config/hoverboard_base.yaml.
#
# Replaces Yahboom's base_node, whose only job was turning the micro-ROS
# board's odom_raw into a TF - the bridge already publishes both.
#
#   ros2 launch hoverboard_base_node base.launch.py
#   ros2 launch hoverboard_base_node base.launch.py params:=/path/to/other.yaml
#
# Most of the time you want robot_bringup instead, which adds the LiDAR
# receiver and the URDF/TF tree on top of this same configuration:
#   ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('hoverboard_base_node'), 'config', 'hoverboard_base.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params,
                              description='Base geometry/limits YAML'),
        Node(
            package='esp_tcp_server',
            executable='hoverboard_udp_bridge_node',
            name='hoverboard_udp_bridge',
            output='screen',
            parameters=[LaunchConfiguration('params')],
        ),
    ])
