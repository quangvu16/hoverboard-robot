#!/usr/bin/env python3
# Explore and map automatically: slam_toolbox (mapping) + Nav2 + frontier explorer.
# Adapted from kaiaai/auto_mapper for the hoverboard robot (ESP32 firmware
# ~/Arduino/ldrobot_ld14p_esp32_ros, bridge hoverboard_udp_bridge_node).
#
#   ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard   # first
#   ros2 launch auto_mapper auto_mapper.launch.py
#   ros2 launch auto_mapper auto_mapper.launch.py map_path:=$HOME/maps/flat rviz:=true
#
# The map is saved after every reached/failed frontier and at the end, as
# <map_path>.yaml/.pgm (for Nav2) and <map_path>.posegraph/.data (slam_toolbox).
#
# Changes from upstream:
#   * real robot: is_sim defaults to false
#   * Nav2 = nav2_bringup navigation_launch.py (no map_server / AMCL - slam_toolbox
#     owns /map and map->odom) with robot_navigation/param/hoverboard.yaml
#     instead of the bundled TurtleBot-sized nav2_params.yaml + bringup copy
#   * slam_toolbox in mapping mode, LD14 ranges (config/mapper_params_online_async.yaml)
#   * resume:=true to continue the last saved pose graph (was automatic)
#   * unreachable frontiers are skipped instead of retried forever
#
# Never run alongside robot_slam / robot_cartographer / navigation2 launches.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('auto_mapper')
    map_path = os.path.expanduser(LaunchConfiguration('map_path').perform(context))
    os.makedirs(os.path.dirname(map_path) or '.', exist_ok=True)
    use_sim_time = LaunchConfiguration('is_sim').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'slam_params_file': os.path.join(share, 'config', 'mapper_params_online_async.yaml'),
                              'map_file_name': map_path,
                              'resume': LaunchConfiguration('resume').perform(context)}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': LaunchConfiguration('params_file').perform(context),
                              'use_sim_time': use_sim_time, 'autostart': 'true'}.items()),
        Node(package='auto_mapper', executable='auto_mapper', name='auto_mapper', output='screen',
             parameters=[{'map_path': map_path,
                          'use_sim_time': use_sim_time == 'true',
                          'min_distance_to_frontier': float(LaunchConfiguration('min_distance_to_frontier').perform(context))}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'config.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value='~/maps/auto_map',
                              description='Where to save the map (no extension)'),
        DeclareLaunchArgument('is_sim', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('resume', default_value='false', choices=['true', 'false'],
                              description='Continue the pose graph saved at map_path'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_navigation'), 'param', 'hoverboard.yaml')),
        DeclareLaunchArgument('min_distance_to_frontier', default_value='1.0'),
        OpaqueFunction(function=_nodes),
    ])
