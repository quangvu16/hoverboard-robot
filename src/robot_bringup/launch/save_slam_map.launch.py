from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import datetime

def launch_setup(context, *args, **kwargs):
    # Get base path and append timestamp
    base_path = LaunchConfiguration('map_path').perform(context)
    #timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    full_path = f"{base_path}"
    #_{timestamp}"

    # Save .pgm and .yaml from /map using map_saver_cli
    map_saver = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', full_path
        ],
        output='screen'
    )

    # Save SLAM pose graph (.data)
    serialize_map = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/slam_toolbox/serialize_map',
            'slam_toolbox/srv/SerializePoseGraph',
            f'{{filename: "{full_path}"}}'
        ],
        output='screen'
    )

    return [
        TimerAction(period=0.5, actions=[map_saver]),
        TimerAction(period=2.5, actions=[serialize_map])
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value='/home/ubuntu/maps/my_map',
            description='Base path for saving map files (timestamp is added)'
        ),
        OpaqueFunction(function=launch_setup)
    ])

