#!/usr/bin/env python3
#
# Cartographer for both mapping and navigation, with Nav2 and RViz on top.
# Cartographer publishes /map and map -> odom in either mode.
#
#   ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard   # first
#
# 1. Mapping (navigate while mapping, no saved map):
#      ros2 launch robot_cartographer cartographer_nav.launch.py
#    Drive or send "Nav2 Goal"s; the map starts at the robot's pose. Save it:
#      ros2 run robot_cartographer save_map ~/m_map
#    (~/m_map.pbstream for step 2, ~/m_map.yaml/.pgm as an image / for AMCL)
#
# 2. Navigation on the saved map (pure localization, map frozen):
#      ros2 launch robot_cartographer cartographer_nav.launch.py map:=$HOME/m_map.pbstream
#    Use "2D Pose Estimate" to say where the robot is (initialpose_relay), or
#    let Cartographer's global search find it, then "Nav2 Goal".
#
# Replaces robot_slam.launch.py (slam_toolbox) and navigation2.launch.py
# (map_server + AMCL). Never run it alongside either: they would all publish
# map -> odom.
#
# LiDAR + IMU only, no wheel encoders: add wheel_odom:=false to BOTH launches
#      ros2 launch robot_bringup robot_bringup.launch.py wheel_odom:=false
#      ros2 launch robot_cartographer cartographer_nav.launch.py wheel_odom:=false [map:=...]
#    Cartographer (robot_2d_imu*.lua) then owns odom -> base_footprint from
#    scan matching + /imu/data, and cartographer_odom turns that TF into
#    /odom/cartographer for Nav2. Mixing the two settings = two publishers of
#    odom -> base_footprint (or none).
#
# Arguments:
#   map:=...pbstream    saved map -> localization mode; empty (default) = mapping
#   wheel_odom:=false   LiDAR + IMU only (see above); true (default) = /odom/filtered
#   rviz:=false         no RViz (e.g. over SSH)
#   nav:=false          Cartographer only, for mapping by teleop
#   params_file:=...    Nav2 parameters, default robot_navigation/param/hoverboard.yaml
#   resolution:=0.05    /map cell size in metres

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def cartographer_nodes(context):
    config_dir = os.path.join(get_package_share_directory('robot_cartographer'), 'config')
    nav2_share = get_package_share_directory('nav2_bringup')
    map_file = os.path.expanduser(LaunchConfiguration('map').perform(context))
    wheel_odom = LaunchConfiguration('wheel_odom').perform(context) == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)

    config = 'robot_2d' if wheel_odom else 'robot_2d_imu'
    if map_file:
        if not os.path.isfile(map_file):
            raise RuntimeError(f'map:={map_file} does not exist (save one with '
                               'ros2 run robot_cartographer save_map ~/m_map)')
        basename = config + '_localization.lua'
        state_args = ['-load_state_filename', map_file, '-load_frozen_state', 'true']
    else:
        basename = config + '.lua'
        state_args = []

    if wheel_odom:
        # Wheel odometry + MPU9250 gyro from the robot_localization EKF
        # (robot_bringup ekf:=true, the default). The IMU reaches Cartographer
        # only through it: use_imu_data stays false so the gyro isn't counted twice.
        remappings = [('odom', 'odom/filtered')]
    else:
        # No odometry topic at all; the IMU goes straight in.
        remappings = [('imu', 'imu/data')]

    nodes = [Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', config_dir,
                   '-configuration_basename', basename] + state_args,
        remappings=remappings,
    )]
    if not wheel_odom:
        # Cartographer's odom -> base_footprint as nav_msgs/Odometry, for Nav2.
        nodes.append(Node(
            package='robot_cartographer',
            executable='cartographer_odom.py',
            name='cartographer_odom',
            output='screen',
        ))
        params_file = RewrittenYaml(
            source_file=params_file,
            param_rewrites={'odom_topic': '/odom/cartographer'},
            convert_types=True)
    if map_file:
        # RViz "2D Pose Estimate" -> restart the localization trajectory there.
        nodes.append(Node(
            package='robot_cartographer',
            executable='initialpose_relay.py',
            name='initialpose_relay',
            output='screen',
            parameters=[{'configuration_directory': config_dir,
                         'configuration_basename': basename,
                         'relative_to_trajectory_id': 0}],
        ))

    # Nav2 without localization (navigation_launch.py, not bringup_launch.py):
    # no map_server, no AMCL.
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav')),
    ))
    return nodes


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')

    rviz = LaunchConfiguration('rviz')
    resolution = LaunchConfiguration('resolution')

    # Rasterizes Cartographer's submaps into nav_msgs/OccupancyGrid on /map
    # (transient_local, which is what Nav2's static_layer subscribes with).
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'resolution': resolution},
                    {'publish_period_sec': 1.0}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav_share, 'rviz', 'robot_navigation2.rviz')],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='Saved .pbstream (ros2 run robot_cartographer save_map): '
                        'localize on it, map frozen. Empty = build a new map.'),
        DeclareLaunchArgument(
            'wheel_odom', default_value='true', choices=['true', 'false'],
            description='false = LiDAR + IMU only (no encoders); needs '
                        'robot_bringup wheel_odom:=false too.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_share, 'param', 'hoverboard.yaml')),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        OpaqueFunction(function=cartographer_nodes),
        occupancy_grid,
        rviz_node,
    ])
