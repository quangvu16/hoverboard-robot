#!/usr/bin/env python3
# Formation: robot1 leads, robot2 and robot3 hold point1 / point2.
#   ros2 launch hoverboard_multi formation_launch.py followers:=robot2
#   ros2 topic pub --once /team std_msgs/msg/String "data: horizontal"
# Drive the leader with hoverboard_ctrl (cmd_vel_topic:=/robot1/cmd_vel) or Nav2.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    leader = LaunchConfiguration('leader').perform(context)
    followers = [f for f in LaunchConfiguration('followers').perform(context).split(',') if f]
    nodes = [Node(package='hoverboard_multi', executable='formation_leader', output='screen',
                  parameters=[{'leader_frame': f'{leader}/base_footprint',
                               'Team': LaunchConfiguration('team').perform(context),
                               'Distance': float(LaunchConfiguration('distance').perform(context))}])]
    for i, f in enumerate(followers[:2]):
        nodes.append(Node(package='hoverboard_multi', executable='formation_follower',
                          name=f'formation_follower_{f}', output='screen',
                          parameters=[{'robot_frame': f'{f}/base_footprint',
                                       'target_frame': f'point{i + 1}',
                                       'cmd_vel_topic': f'/{f}/cmd_vel'}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('leader', default_value='robot1'),
        DeclareLaunchArgument('followers', default_value='robot2'),
        DeclareLaunchArgument('team', default_value='vertical'),
        DeclareLaunchArgument('distance', default_value='0.8'),
        OpaqueFunction(function=_nodes),
    ])
