#!/usr/bin/env python3
# One hoverboard robot of several: bridge + LiDAR receiver + URDF, all in the
# robot's namespace with prefixed frames (robot1/odom, robot1/base_scan, ...).
#
#   ros2 launch hoverboard_multi robot_bringup_multi.launch.py robot_name:=robot1
#   ros2 launch hoverboard_multi robot_bringup_multi.launch.py robot_name:=robot2 \
#        scan_port:=9910 cmd_port:=9911 telem_port:=9912
#
# Every robot's ESP32 needs its OWN ports: set DEST_PORT / HB_CMD_PORT /
# HB_TELEM_PORT in that robot's firmware config.h to match (robot1 keeps
# 9900/9901/9902). Replaces Yahboom's bringup_multi launches, whose IMU filter,
# EKF, MS200 LiDAR and Mcnamu driver a hoverboard does not have.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot(context):
    name = LaunchConfiguration('robot_name').perform(context)
    base_yaml = os.path.join(get_package_share_directory('hoverboard_base_node'),
                             'config', 'hoverboard_base.yaml')
    return [
        Node(package='esp_tcp_server', executable='hoverboard_udp_bridge_node',
             name='hoverboard_udp_bridge', namespace=name, output='screen',
             parameters=[base_yaml, {
                 'odom_frame': f'{name}/odom',
                 'base_frame': f'{name}/base_footprint',
                 'telem_port': int(LaunchConfiguration('telem_port').perform(context)),
                 'cmd_port': int(LaunchConfiguration('cmd_port').perform(context)),
             }]),
        Node(package='esp_tcp_server', executable='udp_lidar_receiver_node',
             name='udp_lidar_receiver', namespace=name, output='screen',
             parameters=[{'port': int(LaunchConfiguration('scan_port').perform(context)),
                          'frame_id': f'{name}/base_scan'}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hoverboard_description'), 'launch', 'description_launch.py')),
            launch_arguments={'robot_name': name}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('scan_port', default_value='9900'),
        DeclareLaunchArgument('cmd_port', default_value='9901'),
        DeclareLaunchArgument('telem_port', default_value='9902'),
        OpaqueFunction(function=_robot),
    ])
