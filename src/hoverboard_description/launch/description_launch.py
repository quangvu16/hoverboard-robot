#!/usr/bin/env python3
# robot_state_publisher for the hoverboard URDF.
#
#   ros2 launch hoverboard_description description_launch.py
#   ros2 launch hoverboard_description description_launch.py robot_name:=robot1
#
# robot_name (default empty) puts the nodes in that namespace and prefixes
# every frame (robot1/base_link, ...), which is how the multi-robot setup tells
# robots apart - one URDF instead of Yahboom's copy per robot.
#
# joint_state_publisher merges the real wheel angles the bridge publishes on
# wheel_joint_states; with no bridge running the wheels just sit at zero.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    name = LaunchConfiguration('robot_name').perform(context).strip('/')
    model = LaunchConfiguration('model').perform(context)
    with open(model, 'r') as f:
        urdf = f.read()
    prefix = (name + '/') if name else ''
    ns = name or None
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{'robot_description': urdf,
                         'frame_prefix': prefix,
                         'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=ns,
            parameters=[{'source_list': ['wheel_joint_states'], 'rate': 30}],
        ),
    ]


def generate_launch_description():
    default_model = os.path.join(
        get_package_share_directory('hoverboard_description'), 'urdf', 'hoverboard.urdf')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Namespace and frame prefix, e.g. robot1'),
        DeclareLaunchArgument('model', default_value=default_model,
                              description='URDF file'),
        OpaqueFunction(function=_nodes),
    ])
