"""
slam_toolbox_lifelong.launch.py
===============================
Starts lifelong_slam_toolbox_node, waits for it to activate,
then runs load_slam_map.py to load & shift the pose-graph.

Launch arguments
----------------
autostart        : configure & activate automatically     (bool, default true)
params_file      : mapper_params_lifelong.yaml to use     (string)
map_file         : *.posegraph file to load               (string)
start_x / y / yaw: start pose in map frame [m|rad]        (double)

Example
-------
ros2 launch robot_bringup slam_toolbox_lifelong.launch.py \
        params_file:=/home/ubuntu/mapper_params_lifelong.yaml \
        map_file:=/home/ubuntu/factory.posegraph \
        start_x:=-1.85 start_y:=2.10 start_yaw:=-1.57
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    # ───────── launch arguments ────────────────────────────────────────
    autostart     = LaunchConfiguration('autostart')
    params_file   = LaunchConfiguration('params_file')
    map_file      = LaunchConfiguration('map_file')
    start_x       = LaunchConfiguration('start_x')
    start_y       = LaunchConfiguration('start_y')
    start_yaw     = LaunchConfiguration('start_yaw')

    decls = [
        DeclareLaunchArgument('autostart',  default_value='true'),
        DeclareLaunchArgument('params_file',
            default_value='/home/ubuntu/slam_toolbox_ws/src/slam_toolbox/config/mapper_params_lifelong.yaml'),
        DeclareLaunchArgument('map_file',
            default_value='/home/ubuntu/my_map.posegraph'),
        DeclareLaunchArgument('start_x',   default_value='0.0'),
        DeclareLaunchArgument('start_y',   default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
    ]

    # ───────── slam_toolbox node (lifelong) ───────────────────────────
    slam_toolbox = LifecycleNode(
        namespace='',
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file])

    # ───────── configure & activate automatically (if desired) ───────
    configure_evt = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == slam_toolbox,
            transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))

    activate_evt = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda action: action == slam_toolbox,
                    transition_id=Transition.TRANSITION_ACTIVATE)),
                LogInfo(msg='slam_toolbox activated')]),
        condition=IfCondition(autostart))

    # ───────── call load_slam_map.py 5 s after launch ─────────────────
    # (enough time for configure→activate; adjust if your machine is slow)
    deserialize_caller = TimerAction(
        period=5.0,
        actions=[Node(
            package='robot_bringup',          # ← your package name
            executable='load_slam_map.py',
            name='load_slam_map',
            output='screen',
            parameters=[{
                'map_file':  map_file,
                'start_x':   start_x,
                'start_y':   start_y,
                'start_yaw': start_yaw
            }])])

    # ───────── build description ─────────────────────────────────────
    return LaunchDescription(
        decls + [
            slam_toolbox,
            configure_evt,
            activate_evt,
            deserialize_caller
        ])
