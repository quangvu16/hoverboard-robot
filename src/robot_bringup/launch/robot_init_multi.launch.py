from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import yaml


def launch_setup(context, *args, **kwargs):
    robots_file = LaunchConfiguration('robots_file').perform(context)
    robot_config = LaunchConfiguration('robot_config').perform(context)

    with open(robots_file, 'r') as f:
        cfg = yaml.safe_load(f)

    actions = []

    for robot in cfg.get('robots', []):
        name = robot['name']

        actions.append(
            Node(
                package='robot_init_multi',
                executable='robot_init_node',
                namespace=name,
                name='robot_init_multi',
                output='screen',
                parameters=[{
                    'config_path': os.path.expanduser(os.path.expandvars(robot_config)),
                }],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file',
            default_value='/home/ubuntu/robot_ESP_ws/src/esp_tcp_server_multi/config/robots.yaml',
            description='Path to robots.yaml'
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='/home/ubuntu/robot_ESP_ws/src/config/robot_config_multi.json',
            description='Path to robot_init multi robot json config'
        ),
        OpaqueFunction(function=launch_setup),
    ])
