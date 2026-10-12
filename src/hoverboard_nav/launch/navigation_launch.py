#!/usr/bin/env python3
# Navigate on a saved map: Nav2 with map_server + AMCL, hoverboard parameters.
#   ros2 launch hoverboard_nav navigation_launch.py map:=$HOME/maps/<name>.yaml
# Then in RViz: "2D Pose Estimate" once, then "Nav2 Goal".
# To navigate WHILE mapping (no saved map) use robot_cartographer instead.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robot_navigation'), 'param', 'hoverboard.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/m_map.yaml'),
                              description='Map YAML to navigate on'),
        DeclareLaunchArgument('params_file', default_value=params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': 'false', 'autostart': 'true'}.items()),
    ])
