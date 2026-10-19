#!/usr/bin/env python3
#
# Robot bring-up: the ESP32 control link, a LiDAR, and the URDF/TF tree.
#
#   ros2 launch robot_bringup robot_bringup.launch.py               # hoverboard
#   ros2 launch robot_bringup robot_bringup.launch.py base:=esp_tcp
#   ros2 launch robot_bringup robot_bringup.launch.py \
#        lidar:=ldlidar bridge_ip:=192.168.1.50 bridge_port:=8889
#
# Two things are selectable, because the ESP32 firmware decides both and this
# machine has carried more than one of them.
#
# The base link - who owns /odom, the odom->base_footprint TF and /cmd_vel:
#
#   base:=esp_tcp     esp_tcp_server_node, the kaiaai length-prefixed JSON
#                     protocol on TCP 9000, where the ESP32 dials in as client.
#                     The original board; no longer the default.
#   base:=hoverboard  hoverboard_udp_bridge_node: "HBT1" telemetry in on UDP
#                     9902, "HBC1" wheel commands out on UDP 9901, plus the
#                     three HC-SR04s: /ultrasonic/{distance_mm,obstacle_near}
#                     (front, also /ultrasonic/center/...) and /ultrasonic/{left,right}/..., and the LEDs
#                     as lit: /lights/{left,right,back}. Spoken by
#                     ~/Arduino/ldrobot_ld14p_esp32_mpu9250_ros; pair it with lidar:=udp.
#                     The default - it is the base on the robot now.
#   base:=none        LiDAR and TF only.
#
# ekf:=true (hoverboard only, the default) fuses /odom with the MPU9250's
# /imu/data in a robot_localization EKF (param/ekf_hoverboard.yaml). The EKF
# then owns the odom->base_footprint TF and the bridge stops publishing it; the
# fused odometry is on /odom/filtered, raw wheel odometry stays on /odom.
# slam_toolbox / AMCL follow the TF; Nav2 (robot_navigation/param/hoverboard.yaml)
# and Cartographer read /odom/filtered, so keep ekf:=true when using them.
#
# Velocity commands (hoverboard only) pass a safety chain before the bridge:
#
#   Nav2 --/cmd_vel--> collision_monitor --/cmd_vel_safe--> twist_mux --/cmd_vel_mux--> bridge
#   joystick / web stick / app / rqt --/cmd_vel_manual (beats Nav2)--^       ^
#   /cmd_vel_lock (std_msgs/Bool, true = hold still, beats everything) ------'
#
# collision_monitor:=true (default) slows Nav2 down / stops it when the LiDAR or
#   the sonars see something just ahead (param/collision_monitor_hoverboard.yaml).
# cmd_mux:=auto (default) runs twist_mux (param/twist_mux_hoverboard.yaml) if
#   ros-humble-twist-mux is installed, and otherwise falls back to the bridge
#   listening to /cmd_vel(_safe) and /cmd_vel_manual itself, last message wins.
#   true insists on twist_mux, false never runs it.
# The sonars also reach Nav2's local costmap as /range/{front,left,right}
# (robot_navigation/param/hoverboard.yaml, range_layer).
#
# wheel_odom:=false (hoverboard only) takes the encoders out of localization:
# no EKF, and the bridge publishes no odom->base_footprint TF (its /odom topic
# stays, unused; the wheels are still driven). Only for
#   ros2 launch robot_cartographer cartographer_nav.launch.py wheel_odom:=false
# where Cartographer owns that TF from LiDAR + IMU. Anything else (slam_toolbox,
# AMCL) would have no odom frame.
#
# Whichever base you pick, /odom and the odom->base_footprint TF are absent or
# frozen at the origin until the ESP32 gets wheel feedback from the motor board -
# which is what slam_toolbox and nav2 need. Watch /esp/link_status (esp_tcp) or
# /hoverboard/link_status (hoverboard) to see whether a board ever arrives.
#
# And the LiDAR source, because this robot has two of those too and only one
# may own /scan at a time:
#
#   lidar:=udp      udp_lidar_receiver_node decodes the "LDS1" datagrams the
#                   ESP32 broadcasts on UDP 9900 (~/Arduino/ld14p_udp_scan).
#                   The default, because that is the firmware on the board.
#   lidar:=ldlidar  the vendor driver (ldlidar_ros2) pulls raw LD14P bytes off
#                   a transparent TCP bridge (~/Arduino/ld14p_tcp_bridge) and
#                   decodes them in the LDROBOT SDK. Needs bridge_ip and
#                   bridge_port; see the warning below.
#   lidar:=none     bring the robot up without a LiDAR.
#
# Either way the scan lands on /scan in frame base_scan, which is what the URDF,
# slam_toolbox and nav2 expect.
#
# bridge_ip is the LiDAR ESP32's own address, never this PC's. Aiming it at this
# machine reaches esp_tcp_server_node on port 9000 instead: the TCP connect
# "succeeds", no LD14P frame ever arrives, the log repeats "ldlidar
# communication is abnormal", and the motor board is locked out of its own port
# meanwhile. Read the address off the bridge's serial console - it prints the
# exact launch line to use.

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _have(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def hoverboard_base(context):
    """The bridge plus the velocity safety chain; their topics depend on each other."""
    if LaunchConfiguration('base').perform(context) != 'hoverboard':
        return []
    param_dir = os.path.join(get_package_share_directory('robot_bringup'), 'param')
    monitor = LaunchConfiguration('collision_monitor').perform(context) == 'true'
    mux_arg = LaunchConfiguration('cmd_mux').perform(context)
    mux = mux_arg == 'true' or (mux_arg == 'auto' and _have('twist_mux'))
    actions = []
    if mux_arg == 'true' and not _have('twist_mux'):
        raise RuntimeError('cmd_mux:=true but twist_mux is not installed: '
                           'sudo apt install ros-humble-twist-mux')
    if mux_arg == 'auto' and not mux:
        actions.append(LogInfo(msg='twist_mux not installed (sudo apt install ros-humble-twist-mux): '
                                   'the bridge takes /cmd_vel and /cmd_vel_manual directly, last message wins'))

    nav_topic = 'cmd_vel_safe' if monitor else 'cmd_vel'
    bridge_in = 'cmd_vel_mux' if mux else nav_topic

    actions.append(Node(
        package='esp_tcp_server',
        executable='hoverboard_udp_bridge_node',
        name='hoverboard_udp_bridge',
        output='screen',
        # Geometry and limits live in ONE file, shared with
        # hoverboard_base_node/launch/base.launch.py. Only min_wheel_rpm can be
        # overridden from the command line (min_wheel_rpm:=...).
        parameters=[
            os.path.join(get_package_share_directory('hoverboard_base_node'),
                         'config', 'hoverboard_base.yaml'),
            {'min_wheel_rpm': float(LaunchConfiguration('min_wheel_rpm').perform(context))},
            # Exactly one node may own odom->base_footprint: the EKF if it
            # runs, Cartographer with wheel_odom:=false, else the bridge.
            {'publish_tf': LaunchConfiguration('ekf').perform(context) != 'true'
                           and LaunchConfiguration('wheel_odom').perform(context) == 'true'},
            # With the mux, manual commands may only arrive through it.
            {'subscribe_cmd_vel_manual': not mux},
        ],
        remappings=[('cmd_vel', bridge_in)],
    ))

    if monitor:
        actions.append(Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[os.path.join(param_dir, 'collision_monitor_hoverboard.yaml')],
        ))
        # collision_monitor is a lifecycle node; this one brings it up and
        # keeps it up, separately from Nav2's own lifecycle managers.
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_safety',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['collision_monitor'],
                         'bond_timeout': 0.0}],
        ))

    if mux:
        actions.append(Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[os.path.join(param_dir, 'twist_mux_hoverboard.yaml')],
            remappings=[('cmd_vel_out', 'cmd_vel_mux'), ('/cmd_vel_out', 'cmd_vel_mux')]
                       + ([] if monitor else [('cmd_vel_safe', 'cmd_vel')]),
        ))

    actions.append(LogInfo(msg='velocity chain: ' + ' -> '.join(
        ['/cmd_vel'] + (['collision_monitor', '/cmd_vel_safe'] if monitor else [])
        + (['twist_mux (+ /cmd_vel_manual, /cmd_vel_lock)', '/cmd_vel_mux'] if mux
           else ['(+ /cmd_vel_manual)']) + ['hoverboard_udp_bridge'])))
    return actions


def generate_launch_description():

    # URDF -> /robot_description and the base_footprint -> ... -> base_scan TF.
    # The hoverboard base uses hoverboard_description's model (with sonar_link,
    # sonar_left_link and sonar_right_link); the others keep
    # robot_description/urdf/robot_esp.urdf.
    rsp_launch = os.path.join(get_package_share_directory('robot_bringup'),
                              'launch', 'robot_state_publisher.launch.py')
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        condition=LaunchConfigurationNotEquals('base', 'hoverboard'),
    )
    robot_state_hoverboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        launch_arguments={'model': os.path.join(
            get_package_share_directory('hoverboard_description'),
            'urdf', 'hoverboard.urdf')}.items(),
        condition=LaunchConfigurationEquals('base', 'hoverboard'),
    )

    robot_joint = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            # Wheel angles from hoverboard_udp_bridge; joints nobody reports
            # (all of them with base:=esp_tcp) stay at 0 as before.
            'source_list': ['/wheel_joint_states'],
            'rate': 30,
        }]
    )

    # /odom + odom->base_footprint TF from wheel ticks, and /cmd_vel ->
    # per-wheel RPM. Binds UDP 9902 and answers whoever sent the telemetry, so
    # the board's address never has to be configured here. The firmware is
    # ~/Arduino/ldrobot_ld14p_esp32_mpu9250_ros; it also carries the front, left and
    # right HC-SR04 ranges (/ultrasonic/*, /ultrasonic/center/*, /ultrasonic/left/*,
    # /ultrasonic/right/*) and the LED states (/lights/left, /lights/right,
    # /lights/back). The avoidance reflex and the turn/stop lights run on the
    # ESP32 itself; those topics only report what it does.
    base_hoverboard = OpaqueFunction(function=hoverboard_base)

    # Wheel odometry + MPU9250 gyro -> odom->base_footprint (ekf:=true).
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('robot_bringup'),
                                 'param', 'ekf_hoverboard.yaml')],
        remappings=[('odometry/filtered', 'odom/filtered')],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('ekf'), "' == 'true' and '",
             LaunchConfiguration('wheel_odom'), "' == 'true' and '",
             LaunchConfiguration('base'), "' == 'hoverboard'"])),
    )

    # The original base: same ROS surface as the hoverboard bridge plus the
    # ultrasonic topics, spoken over TCP 9000 with the ESP32 as client.
    base_esp_tcp = Node(
        package='esp_tcp_server',
        executable='esp_tcp_server_node',
        name='esp_tcp_server_lenprefix',
        output='screen',
        condition=LaunchConfigurationEquals('base', 'esp_tcp')
    )

    lidar_udp = Node(
        package='esp_tcp_server',
        executable='udp_lidar_receiver_node',
        name='udp_lidar_receiver',
        output='screen',
        condition=LaunchConfigurationEquals('lidar', 'udp')
    )

    # Resolved lazily through FindPackageShare, so a missing ldlidar_ros2
    # overlay only breaks lidar:=ldlidar, not the default bring-up.
    lidar_ldlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ldlidar_ros2'),
                'launch',
                'ld14p_network.launch.py'
            ])
        ),
        launch_arguments={
            'bridge_ip': LaunchConfiguration('bridge_ip'),
            'bridge_port': LaunchConfiguration('bridge_port'),
            'frame_id': 'base_scan',
            'topic_name': 'scan',
            # robot_state_publisher already owns base_link -> base_scan.
            'publish_static_tf': 'false',
        }.items(),
        condition=LaunchConfigurationEquals('lidar', 'ldlidar')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'base', default_value='hoverboard',
            choices=['esp_tcp', 'hoverboard', 'none'],
            description='Which node owns /odom and /cmd_vel: hoverboard (HBT1/HBC1 '
                        'over UDP 9902/9901, ldrobot_ld14p_esp32_mpu9250_ros, the default), '
                        'esp_tcp (kaiaai JSON over TCP 9000), or none.'),
        DeclareLaunchArgument(
            'min_wheel_rpm', default_value='0.0',
            description='base:=hoverboard only - non-zero wheel commands are '
                        'lifted to at least this many rpm (0 = off).'),
        DeclareLaunchArgument(
            'wheel_odom', default_value='true', choices=['true', 'false'],
            description='base:=hoverboard only - false: encoders not used for '
                        'localization (no EKF, no bridge odom TF), for '
                        'robot_cartographer cartographer_nav.launch.py wheel_odom:=false.'),
        DeclareLaunchArgument(
            'ekf', default_value='true', choices=['true', 'false'],
            description='base:=hoverboard only - fuse /odom and the MPU9250 '
                        '/imu/data in robot_localization; the EKF then owns '
                        'the odom->base_footprint TF.'),
        DeclareLaunchArgument(
            'lidar', default_value='udp',
            choices=['udp', 'ldlidar', 'none'],
            description='Which driver publishes /scan: udp (LDS1 datagrams on '
                        'UDP 9900), ldlidar (LD14P bytes over the TCP bridge), '
                        'or none.'),
        DeclareLaunchArgument(
            'bridge_ip', default_value='192.168.1.50',
            description='lidar:=ldlidar only - the LiDAR ESP32 bridge address. '
                        'Must be the board, not this PC.'),
        DeclareLaunchArgument(
            'bridge_port', default_value='8889',
            description='lidar:=ldlidar only - TCP port of the bridge, '
                        'BRIDGE_PORT in the sketch. Not 9000, that is '
                        'esp_tcp_server.'),

        lidar_udp,
        lidar_ldlidar,
        robot_state,
        robot_state_hoverboard,
        robot_joint,
        DeclareLaunchArgument(
            'collision_monitor', default_value='true', choices=['true', 'false'],
            description='base:=hoverboard only - Nav2 collision_monitor between '
                        '/cmd_vel and the wheels: slow down / stop from the LiDAR '
                        'and sonars just ahead.'),
        DeclareLaunchArgument(
            'cmd_mux', default_value='auto', choices=['auto', 'true', 'false'],
            description='base:=hoverboard only - twist_mux arbitration: manual '
                        '(/cmd_vel_manual) over Nav2, /cmd_vel_lock holds still. '
                        'auto = if ros-humble-twist-mux is installed.'),

        base_hoverboard,
        ekf,
        base_esp_tcp,
    ])
