from os.path import isfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file_name = LaunchConfiguration('map_file_name')

    map_file_name_value = LaunchConfiguration('map_file_name').perform(context)

    params = {'use_sim_time': use_sim_time}
    # Resume the previous run's pose graph only when asked: slam_toolbox then
    # assumes the robot starts at that map's origin, which is wrong once it
    # has been carried somewhere else. Upstream resumed whenever a file existed.
    resume = LaunchConfiguration('resume').perform(context) == 'true'
    if resume and isfile(map_file_name_value + '.posegraph'):
        params['map_file_name'] = map_file_name
    return [Node(
        parameters=[
            slam_params_file,
            params
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')]


def generate_launch_description():
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_map_file_cmd = DeclareLaunchArgument('map_file_name', default_value='')
    declare_resume_cmd = DeclareLaunchArgument('resume', default_value='false')

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_slam_params_file_cmd,
        declare_map_file_cmd,
        declare_resume_cmd,
        # start_async_slam_toolbox_node,
        OpaqueFunction(function=launch_setup)
    ])
