from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

import os
import yaml


def _robot_description_for(name: str):
    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'robot_esp_multi.urdf.xacro',
    ])

    return Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'prefix:=',
        name,
        '/',
    ])


def launch_setup(context, *args, **kwargs):
    robots_file = LaunchConfiguration('robots_file').perform(context)
    go_to_goal_params = LaunchConfiguration('go_to_goal_params').perform(context)

    with open(robots_file, 'r') as f:
        cfg = yaml.safe_load(f)

    actions = []

    for robot in cfg.get('robots', []):
        name = robot['name']

        robot_description = _robot_description_for(name)

        esp_config = os.path.expanduser(
            os.path.expandvars(robot['esp_config'])
        )

        use_sim_time = bool(robot.get('use_sim_time', False))

        odom_frame = robot.get('odom_frame', f'{name}/odom')
        base_frame = robot.get('base_frame', f'{name}/base_footprint')
        lidar_frame = robot.get('lidar_frame', f'{name}/base_scan')

        # 1) Robot State Publisher
        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=name,
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                }],
            )
        )

        # 2) Joint State Publisher
        actions.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=name,
                name='joint_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                }],
            )
        )

        # 3) ESP TCP bridge
        actions.append(
            Node(
                package='esp_tcp_server_multi',
                executable='esp_tcp_server_node',
                namespace=name,
                name='esp_node',
                output='screen',
                parameters=[{
                    'config_path': esp_config,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'use_sim_time': use_sim_time,
                }],
            )
        )

        # 4) UDP LiDAR receiver
        actions.append(
            Node(
                package='esp_tcp_server_multi',
                executable='udp_lidar_receiver_node',
                namespace=name,
                name='udp_lidar_receiver',
                output='screen',
                parameters=[{
                    'port': int(robot['lidar_port']),
                    'frame_id': lidar_frame,
                    'flip_lr': bool(robot.get('flip_lr', True)),
                    'angle0_deg': float(robot.get('angle0_deg', 180.0)),
                    'use_sim_time': use_sim_time,
                }],
            )
        )

        # 5) Go-to-goal controller
        # actions.append(
        #     Node(
        #         package='robot_service',
        #         executable='robot_trajectory',
        #         namespace=name,
        #         name='robot_trajectory',
        #         output='screen',
        #         parameters=[
        #             go_to_goal_params,
        #             {
        #                 'use_sim_time': use_sim_time,

        #                 # TF frames
        #                 'path_frame': 'map',
        #                 'odom_frame': odom_frame,
        #                 'base_frame': base_frame,
        #             }
        #         ],
        #     )
        # )

        actions.append(
            Node(
                package="robot_service",
                executable="waypoint_pure_pursuit",
                namespace=name,
                name="waypoint_pure_pursuit",
                output="screen",
                parameters=[
                    go_to_goal_params,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                    },
                ],
            )
        )

    return actions


def generate_launch_description():
    default_robots_file = (
        '/home/ubuntu/robot_ESP_ws/src/'
        'esp_tcp_server_multi/config/robots.yaml'
    )

    default_go_to_goal_params = (
        '/home/ubuntu/robot_ESP_ws/src/config/waypoint_pure_pursuit.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file',
            default_value=default_robots_file,
            description='Path to robots.yaml'
        ),

        DeclareLaunchArgument(
            'go_to_goal_params',
            default_value=default_go_to_goal_params,
            description='Path to go_to_goal_diff_drive_tf params yaml'
        ),

        OpaqueFunction(function=launch_setup),
    ])
