import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    forbidden_yaml_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value='/home/agv/robot_gui/src/robot_slam_gui/task/aeon_1/forbidden_zones.yaml',
        description='YAML path for forbidden zones'
    )

    robot_forbidden = Node(
        package='robot_service',
        executable='robot_forbidden',
        name='robot_forbidden',
        output='screen',
        parameters=[{
            'yaml_path': LaunchConfiguration('yaml_path')
        }]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'nav_localization_launch.py'
            )
        )
    )

    lidar_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )
    robot_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_navigation'),
                'launch',
                'navigation2.launch.py'
            )
        )
    )

    robot_safety = Node(
        package='robot_service',
        executable='robot_safety',
        name='robot_safety',
        output='screen'
    )

    robot_barcode = Node(
        package='robot_service',
        executable='barcode_reader_node',
        name='barcode_reader_node',
        output='screen'
    )

    robot_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/home/agv/robot_slam_ws/src/config/ekf.yaml']
    )

    robot_marker = Node(
        package='markers_algorithm',
        executable='markers_algorithm_node',
        name='markers_algorithm_node',
        output='screen',
    )

    robot_filter_scan = Node(
        package='robot_service',
        executable='robot_filter_scan',
        name='robot_filter_scan',
        output='screen',
        parameters=[{
            'range_diff_threshold': 0.3,
            'window_size': 3,
        }]
    )

    return LaunchDescription([
        forbidden_yaml_arg,
        robot_forbidden,
        lidar_safety,
        robot_navigation,
        robot_safety,
        robot_barcode,
        slam_toolbox,
        robot_ekf,
        robot_marker,
        robot_filter_scan
    ])
