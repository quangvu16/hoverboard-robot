"""Auto-docking: the dock manager (robot_docking/dock_manager.py).

  ros2 launch robot_docking docking.launch.py

Needs Nav2 running (it drives to the staging pose with navigate_to_pose) and
a map frame: start it with robot_cartographer cartographer_nav or
robot_navigation navigation2, or let robot_web_nav's web_nav.launch.py
include it (docking:=true, the default there).

Arguments:
  auto_dock:=true|false   dock by itself when the ESP32 reports battery low
  dock_file:=PATH         where the saved dock pose lives (~/.ros/robot_dock.json)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto_dock', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('dock_file', default_value='~/.ros/robot_dock.json'),
        Node(
            package='robot_docking',
            executable='dock_manager',
            name='dock_manager',
            output='screen',
            parameters=[{'auto_dock': LaunchConfiguration('auto_dock'),
                         'dock_file': LaunchConfiguration('dock_file')}],
        ),
    ])
