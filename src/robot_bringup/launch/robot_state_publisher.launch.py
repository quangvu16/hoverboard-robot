import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# model: URDF file to publish. Default robot_description/urdf/robot_esp.urdf
# (the kaiaai/esp_tcp base); robot_bringup passes
# hoverboard_description/urdf/hoverboard.urdf for base:=hoverboard.
def _rsp(context):
    model = LaunchConfiguration('model').perform(context)
    print("urdf_file_name : {}".format(model))
    with open(model, 'r') as infp:
        robot_desc = infp.read()
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def generate_launch_description():
    default_model = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'robot_esp.urdf')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('model', default_value=default_model,
                              description='URDF file to publish'),
        OpaqueFunction(function=_rsp),
    ])
