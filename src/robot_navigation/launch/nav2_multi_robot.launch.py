#!/usr/bin/env python3
import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ['1', 'true', 'yes', 'on']


def _robot_names_from_yaml(robots_file: str):
    with open(robots_file, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    return [r['name'] for r in cfg.get('robots', [])]


def _make_robot_params_file(base_params_file: str, ns: str, use_sim_time_value: bool) -> str:
    with open(base_params_file, 'r') as f:
        p = yaml.safe_load(f) or {}

    base_frame = f'{ns}/base_link'
    odom_frame = f'{ns}/odom'

    def rp(node_name):
        p.setdefault(node_name, {})
        p[node_name].setdefault('ros__parameters', {})
        return p[node_name]['ros__parameters']

    # AMCL
    amcl = rp('amcl')
    amcl['use_sim_time'] = use_sim_time_value
    amcl['global_frame_id'] = 'map'
    amcl['odom_frame_id'] = odom_frame
    amcl['base_frame_id'] = base_frame
    amcl['scan_topic'] = f'/{ns}/scan'
    amcl['tf_broadcast'] = True

    # BT navigator
    bt = rp('bt_navigator')
    bt['use_sim_time'] = use_sim_time_value
    bt['global_frame'] = 'map'
    bt['robot_base_frame'] = base_frame
    bt['odom_topic'] = 'odom'

    for node in [
        'controller_server',
        'planner_server',
        'smoother_server',
        'waypoint_follower',
        'velocity_smoother',
        'bt_navigator_navigate_through_poses_rclcpp_node',
        'bt_navigator_navigate_to_pose_rclcpp_node',
    ]:
        rp(node)['use_sim_time'] = use_sim_time_value

    # Local costmap
    p.setdefault('local_costmap', {}).setdefault('local_costmap', {}).setdefault('ros__parameters', {})
    local = p['local_costmap']['local_costmap']['ros__parameters']
    local['use_sim_time'] = use_sim_time_value
    local['global_frame'] = odom_frame
    local['robot_base_frame'] = base_frame

    if 'static_layer' in local:
        local['static_layer']['map_topic'] = '/map'
        local['static_layer']['map_subscribe_transient_local'] = True

    if 'voxel_layer' in local:
        local['voxel_layer'].setdefault('scan', {})['topic'] = 'scan'

    if 'obstacle_layer' in local:
        local['obstacle_layer'].setdefault('scan', {})['topic'] = 'scan'

    # Global costmap
    p.setdefault('global_costmap', {}).setdefault('global_costmap', {}).setdefault('ros__parameters', {})
    global_cm = p['global_costmap']['global_costmap']['ros__parameters']
    global_cm['use_sim_time'] = use_sim_time_value
    global_cm['global_frame'] = 'map'
    global_cm['robot_base_frame'] = base_frame

    if 'static_layer' in global_cm:
        global_cm['static_layer']['map_topic'] = '/map'
        global_cm['static_layer']['map_subscribe_transient_local'] = True

    if 'obstacle_layer' in global_cm:
        global_cm['obstacle_layer'].setdefault('scan', {})['topic'] = 'scan'

    # Behavior server
    behavior = rp('behavior_server')
    behavior['use_sim_time'] = use_sim_time_value
    behavior['global_frame'] = odom_frame
    behavior['robot_base_frame'] = base_frame

    # Velocity smoother
    vs = rp('velocity_smoother')
    vs['use_sim_time'] = use_sim_time_value
    vs['odom_topic'] = 'odom'

    wrapped = {
        ns: p
    }

    out_path = os.path.join(tempfile.gettempdir(), f'nav2_params_{ns}.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(wrapped, f, sort_keys=False)

    return out_path


def _robot_nav_group(ns: str, robot_params_file: str, use_sim_time, autostart):
    return GroupAction([
        PushRosNamespace(ns),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[robot_params_file],
            remappings=[
                ('map', '/map'),
                ('/initialpose', 'initialpose'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['amcl'],
            }],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[robot_params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
            ],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[robot_params_file],
            remappings=[
                ('map', '/map'),
            ],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[robot_params_file],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[robot_params_file],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[robot_params_file],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[robot_params_file],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[robot_params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ])


def launch_setup(context, *args, **kwargs):
    pkg_nav = get_package_share_directory('robot_navigation')

    robots_file = LaunchConfiguration('robots_file').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    nav2_params = LaunchConfiguration('params_file').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_sim_time_value = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    use_rviz = _as_bool(LaunchConfiguration('use_rviz').perform(context))

    actions = []

    actions.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            }
        ],
    ))

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    ))

    for ns in _robot_names_from_yaml(robots_file):
        robot_params_file = _make_robot_params_file(
            nav2_params,
            ns,
            use_sim_time_value
        )
        actions.append(_robot_nav_group(ns, robot_params_file, use_sim_time, autostart))

    if use_rviz:
        rviz_config = os.path.join(pkg_nav, 'rviz', 'robot_navigation2.rviz')
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return actions


def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file',
            default_value='/home/ubuntu/robot_ESP_ws/src/esp_tcp_server_multi/config/robots.yaml',
            description='Path to robots.yaml. Each robot needs a name field: robot_1, robot_2, ...',
        ),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav, 'config', 'my_map.yaml'),
            description='Shared map yaml for all robots',
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value='/home/ubuntu/robot_ESP_ws/src/robot_navigation/config/nav2_multi_params.yaml',
            description='Base Nav2 parameter file',
        ),

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        OpaqueFunction(function=launch_setup),
    ])
