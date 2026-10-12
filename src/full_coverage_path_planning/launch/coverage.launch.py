#!/usr/bin/env python3
# Full coverage of a saved map (ROS 2 port of globaledgesoft's
# Full-Coverage-Path-Planning-on-TurtleBot3-Qualcomm-RB5, rb5_FCPP.launch).
#
#   ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard   # first
#   ros2 launch full_coverage_path_planning coverage.launch.py map:=$HOME/maps/home.yaml
#
# Starts Nav2 (map_server + AMCL) with robot_navigation/param/hoverboard.yaml
# plus config/coverage.yaml (SpiralSTC planner, Regulated Pure Pursuit
# "FollowCoverage" controller), the coverage runner, coverage progress, RViz.
#
# Place the robot, give AMCL a rough "2D Pose Estimate" in RViz, and the runner
# does a slow localization turn and starts covering after start_delay seconds.
# auto_start:=false waits for:  ros2 service call /coverage_runner/start std_srvs/srv/Trigger
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def merge(base, extra):
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            merge(base[k], v)
        else:
            base[k] = v
    return base


def _launch(context):
    share = get_package_share_directory('full_coverage_path_planning')
    with open(LaunchConfiguration('params_file').perform(context)) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(share, 'config', 'coverage.yaml')) as f:
        merge(params, yaml.safe_load(f))
    sweep = float(LaunchConfiguration('tool_radius').perform(context))
    params['planner_server']['ros__parameters']['SpiralSTC']['tool_radius'] = sweep
    tmp = tempfile.NamedTemporaryFile('w', prefix='nav2_coverage_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map').perform(context),
                              'params_file': tmp.name,
                              'use_sim_time': 'false', 'autostart': 'true'}.items()),
        Node(package='full_coverage_path_planning', executable='coverage_runner', output='screen',
             parameters=[{'auto_start': LaunchConfiguration('auto_start').perform(context) == 'true',
                          'start_delay': float(LaunchConfiguration('start_delay').perform(context)),
                          'localize_spin_deg': float(LaunchConfiguration('localize_spin_deg').perform(context))}]),
        Node(package='full_coverage_path_planning', executable='coverage_progress', output='screen',
             parameters=[{'coverage_radius': sweep}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'coverage.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/m_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_navigation'), 'param', 'hoverboard.yaml')),
        DeclareLaunchArgument('tool_radius', default_value='0.165',
                              description='half the swept width (the original sweeper_size/2)'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('start_delay', default_value='15.0',
                              description='seconds to set the AMCL pose before the run starts'),
        DeclareLaunchArgument('localize_spin_deg', default_value='360.0',
                              description='localization turn before planning; 0 = none'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch),
    ])
