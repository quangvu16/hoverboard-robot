#!/usr/bin/env python3
# Nav2 (map_server + AMCL) for one robot of several, on a shared map.
#
#   ros2 launch hoverboard_multi robot_nav_multi.launch.py robot_name:=robot1 map:=$HOME/maps/home.yaml
#
# Parameters come from robot_navigation/param/hoverboard.yaml, rewritten at
# launch time for this robot: frames get the robot prefix (robot1/base_link,
# robot1/odom), topics become relative (scan, odom -> /robot1/scan, ...), the
# map frame stays shared ("map"). So there is no second copy to keep in step -
# Yahboom kept a hand-edited file per robot.
#
# Set each robot's start pose with RViz "2D Pose Estimate" on its own
# /robotN/initialpose, or: ros2 topic pub --once /robot1/initialpose ...
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FRAME_KEYS = {'robot_base_frame', 'base_frame_id', 'odom_frame_id', 'global_frame', 'local_frame'}
ROBOT_FRAMES = {'base_link', 'base_footprint', 'odom'}


def rewrite(node, name):
    if isinstance(node, dict):
        out = {}
        for k, v in node.items():
            if k in FRAME_KEYS and v in ROBOT_FRAMES:
                out[k] = f'{name}/{v}'
            elif k in ('topic', 'odom_topic', 'scan_topic') and isinstance(v, str):
                out[k] = v.lstrip('/')          # relative -> inside the namespace
            else:
                out[k] = rewrite(v, name)
        return out
    if isinstance(node, list):
        return [rewrite(x, name) for x in node]
    return node


def _nav(context):
    name = LaunchConfiguration('robot_name').perform(context)
    with open(LaunchConfiguration('params_file').perform(context)) as f:
        params = rewrite(yaml.safe_load(f), name)
    tmp = tempfile.NamedTemporaryFile('w', prefix=f'nav2_{name}_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()
    return [
        Node(package='hoverboard_multi', executable='tf_relay', namespace=name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'namespace': name, 'use_namespace': 'true',
                              'map': LaunchConfiguration('map').perform(context),
                              'params_file': tmp.name,
                              'use_sim_time': 'false', 'autostart': 'true'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/m_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_navigation'), 'param', 'hoverboard.yaml')),
        OpaqueFunction(function=_nav),
    ])
