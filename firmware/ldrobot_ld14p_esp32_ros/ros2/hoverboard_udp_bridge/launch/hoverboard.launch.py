import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    esp32_ip = LaunchConfiguration('esp32_ip').perform(context)

    parameters = [params_file]
    # Only append the override when one was actually given. Appending an empty
    # string unconditionally would win over the YAML and blank the address,
    # since later parameter entries override earlier ones.
    if esp32_ip:
        parameters.append({'esp32_ip': esp32_ip})

    return [Node(
        package='hoverboard_udp_bridge',
        executable='hoverboard_udp_node',
        name='hoverboard_udp_bridge',
        output='screen',
        parameters=parameters,
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom'),
            ('joint_states', '/joint_states'),
            ('battery_state', '/battery_state'),
        ],
    )]


def generate_launch_description():
    share = get_package_share_directory('hoverboard_udp_bridge')
    default_params = os.path.join(share, 'config', 'hoverboard.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML of hoverboard_udp_bridge parameters'),
        DeclareLaunchArgument(
            'esp32_ip', default_value='',
            description='Override the ESP32 address without editing the YAML'),
        OpaqueFunction(function=launch_setup),
    ])
