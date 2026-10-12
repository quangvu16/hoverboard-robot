import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch configuration for simulation time and map file
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration(
        'map',
        default='/absolute/path/to/map.yaml')  # Replace with your map.yaml path

    return LaunchDescription([
        # Set environment variables to handle RMW implementation
        #SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # Declare the map argument
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to the map file'),

        # Declare the use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Lifecycle manager to handle transitions
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,  # Automatically transition to 'active'
                'node_names': ['map_server']
            }]
        ),

        # Launch the map_server node to publish the map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time
            }],
        ),
    ])
