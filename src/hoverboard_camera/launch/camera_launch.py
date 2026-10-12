#!/usr/bin/env python3
# Camera for the vision packages.
#   ros2 launch hoverboard_camera camera_launch.py source:=0
#   ros2 launch hoverboard_camera camera_launch.py source:=http://192.168.1.50:81/stream view:=true
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='0',
                              description='device index or stream URL'),
        DeclareLaunchArgument('view', default_value='false'),
        Node(package='hoverboard_camera', executable='camera_source', output='screen',
             parameters=[{'source': LaunchConfiguration('source')}]),
        Node(package='hoverboard_camera', executable='sub_img', condition=IfCondition(LaunchConfiguration('view'))),
    ])
