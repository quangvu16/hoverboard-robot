from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_lifelong.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='my_first_map'),
        DeclareLaunchArgument('map_path', default_value='/home/ubuntu/maps/my_first_map'),

        Node(
            package='slam_toolbox',
            executable='lifelong_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_path, {'use_sim_time': True}]
        )
    ])

