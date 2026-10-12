#!/usr/bin/env python3
# Autonomous frontier exploration on the hoverboard robot: slam_toolbox
# (mapping) + Nav2 (hoverboard.yaml, no map_server/AMCL) + explore_lite.
#
#   ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard   # first
#   ros2 launch explore_lite explore_hoverboard.launch.py rviz:=true
#   ros2 launch explore_lite explore_hoverboard.launch.py save_map:=$HOME/maps/flat
#
# Pause / continue:  ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: false"
# When exploration ends the map is saved to save_map (.yaml/.pgm) if given;
# otherwise: ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>
#
# Never run alongside robot_slam / robot_cartographer / navigation2 / auto_mapper.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('explore_lite')
    save_map = os.path.expanduser(LaunchConfiguration('save_map').perform(context))
    nodes = [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen',
             parameters=[os.path.join(share, 'config', 'slam_mapping.yaml'), {'use_sim_time': False}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': LaunchConfiguration('params_file').perform(context),
                              'use_sim_time': 'false', 'autostart': 'true'}.items()),
        Node(package='explore_lite', executable='explore', name='explore_node', output='screen',
             parameters=[os.path.join(share, 'config', 'params.yaml'), {'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(get_package_share_directory('robot_navigation'),
                                           'rviz', 'robot_navigation2.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    if save_map:
        os.makedirs(os.path.dirname(save_map) or '.', exist_ok=True)
        nodes.append(Node(package='explore_lite', executable='save_map_on_finish.py',
                          output='screen', parameters=[{'map_path': save_map}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_navigation'), 'param', 'hoverboard.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('save_map', default_value='',
                              description='Save the map here (no extension) when exploration ends'),
        OpaqueFunction(function=_nodes),
    ])
