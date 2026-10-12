from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _resolve_path(path: str) -> str:
    return os.path.expanduser(os.path.expandvars(path))


def launch_setup(context, *args, **kwargs):
    robots_file = _resolve_path(LaunchConfiguration('robots_file').perform(context))
    default_urdf_xacro = _resolve_path(LaunchConfiguration('urdf_xacro').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')

    with open(robots_file, 'r') as f:
        cfg = yaml.safe_load(f) or {}

    actions = []

    for robot in cfg.get('robots', []):
        # name chính là namespace của robot: robot_1, robot_2, ...
        name = robot['name']
        prefix = robot.get('prefix', f'{name}/')

        # Cho phép mỗi robot dùng URDF riêng nếu cần, không thì dùng urdf_xacro mặc định
        urdf_xacro = _resolve_path(robot.get('urdf_xacro', default_urdf_xacro))

        robot_description = Command([
            'xacro ', urdf_xacro,
            ' prefix:=', prefix,
        ])

        # 1) Robot State Publisher riêng cho từng robot
        # TF frame trong URDF đã được prefix: robot_1/base_link, robot_1/base_scan, ...
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

        # 2) Joint State Publisher riêng cho từng robot
        # Node này publish joint_states trong namespace tương ứng: /robot_1/joint_states
        actions.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=name,
                name='joint_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
            )
        )

        # 3) ESP TCP server node riêng theo namespace robot
        esp_config = _resolve_path(robot['esp_config'])
        actions.append(
            Node(
                package='esp_tcp_server_multi',
                executable='esp_tcp_server_node',
                namespace=name,
                name='esp_node',
                output='screen',
                parameters=[{
                    'config_path': esp_config,
                }],
            )
        )

        # 4) Lidar receiver riêng theo namespace robot
        # Topic scan sẽ là /robot_1/scan, /robot_2/scan, ...
        # frame_id phải khớp với frame trong URDF đã prefix: robot_1/base_scan, ...
        actions.append(
            Node(
                package='esp_tcp_server_multi',
                executable='udp_lidar_receiver_node',
                namespace=name,
                name='udp_lidar_receiver',
                output='screen',
                parameters=[{
                    'port': int(robot['lidar_port']),
                    'frame_id': robot.get('lidar_frame', f'{name}/base_scan'),
                    'flip_lr': bool(robot.get('flip_lr', True)),
                    'angle0_deg': float(robot.get('angle0_deg', 180.0)),
                }],
            )
        )

    return actions


def generate_launch_description():
    default_robots_file = '/home/ubuntu/robot_ESP_ws/src/esp_tcp_server_multi/config/robots.yaml'

    # Đổi path này nếu bạn đặt file xacro ở chỗ khác.
    # Khuyến nghị copy robot_esp_prefixed.urdf.xacro vào package robot_description/urdf/
    default_urdf_xacro = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'robot_esp_multi.urdf.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file',
            default_value=default_robots_file,
            description='Path to robots.yaml'
        ),
        DeclareLaunchArgument(
            'urdf_xacro',
            default_value=default_urdf_xacro,
            description='Path to prefixed robot URDF xacro'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        OpaqueFunction(function=launch_setup),
    ])
