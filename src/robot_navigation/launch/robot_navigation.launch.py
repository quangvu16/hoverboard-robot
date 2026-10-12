import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map')

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('robot_navigation'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('robot_navigation'),
                'maps',
                'my_map.yaml'),  # Ensure this is correct
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Start Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        # Ensure Map Server is up before SLAM Toolbox (Delay SLAM Toolbox by 3 seconds)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='localization_slam_toolbox_node',
                    name='slam_toolbox_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'localization_mode': True,  # Enables localization mode
                        'map_file_name': map_file,  # Corrected passing of map_file
                        'mode': 'localization',
                        'map_topic': '/map',  # Ensure correct subscription
                        'map_frame': 'map',
                        'base_frame': 'base_link',
                        'odom_frame': 'odom',
                        'scan_topic': '/scan'
                    }]
                ),
            ]
        ),

        # Include Nav2 Bringup (Navigation) - Ensure AMCL is removed from bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
