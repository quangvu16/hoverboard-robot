import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    robot_task_executor = Node(
        package='robot_service',
        executable='robot_task_executor',
        name='robot_task_executor',
        output='screen'
    )

    robot_navigate_to_pose = Node(
        package='robot_service',
        executable='robot_navigate_to_pose',
        name='robot_navigate_to_pose',
        output='screen'
    )

    robot_navigate_waypoint = Node(
        package='robot_service',
        executable='robot_navigate_waypoint',
        name='robot_navigate_waypoint',
        output='screen'
    )

    robot_action = Node(
        package='robot_service',
        executable='robot_action',
        name='robot_action',
        output='screen'
    )
    return LaunchDescription([
        robot_task_executor,
        robot_navigate_to_pose,
        robot_navigate_waypoint,
        robot_action

    ])
