#!/usr/bin/env python3
#
# LD14P reached over the network instead of a serial port.
#
# The LiDAR stays wired to an ESP32, which runs a transparent bridge: it copies
# the LiDAR's UART bytes onto a TCP socket and nothing else. All the decoding -
# packet framing, CRC8, angle interpolation, the ranging-centre correction -
# happens here in the LDROBOT SDK, exactly as it would over /dev/ttyUSB0.
#
#   ESP32 sketch: ~/Arduino/ld14p_tcp_bridge/ld14p_tcp_bridge.ino
#
# comm_mode picks who dials whom:
#   tcp_client  this node connects out to the ESP32 (the default). The ESP32
#               listens, so it needs a fixed address - the sketch sets a static
#               IP for that reason. Reconnects cleanly after either side
#               reboots, which is why it is the default.
#   tcp_server  this node binds server_ip:server_port and waits in accept()
#               for the ESP32 to dial in. server_ip must then be a local
#               address ("0.0.0.0" for all interfaces). The SDK does not set
#               SO_REUSEADDR, so a relink can fail with "Address already in
#               use" until the old connection leaves TIME_WAIT.
#
# Heads up, two of them:
#   * Connect() blocks. In tcp_client mode the node sits in connect() until the
#     ESP32 answers or the OS gives up; ctrl-c during that wait is only handled
#     once the call returns (ros2 launch escalates to SIGTERM, so it does die).
#   * This publishes /scan, and so does udp_lidar_receiver_node in
#     robot_bringup.launch.py. Run one or the other, never both.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  bridge_ip = LaunchConfiguration('bridge_ip')
  bridge_port = LaunchConfiguration('bridge_port')
  comm_mode = LaunchConfiguration('comm_mode')
  product_name = LaunchConfiguration('product_name')
  frame_id = LaunchConfiguration('frame_id')
  topic_name = LaunchConfiguration('topic_name')
  publish_static_tf = LaunchConfiguration('publish_static_tf')

  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld14p',
      output='screen',
      parameters=[
        {'product_name': product_name},
        {'laser_scan_topic_name': topic_name},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': frame_id},
        {'comm_mode': comm_mode},
        # Both are strings to the SDK. Without the explicit value_type, launch
        # infers int from a digits-only argument and the node aborts with
        # InvalidParameterTypeException on server_port.
        {'server_ip': ParameterValue(bridge_ip, value_type=str)},
        {'server_port': ParameterValue(bridge_port, value_type=str)},
        {'auto_reconnect': True},
        # Unused in network mode, but the node still declares them.
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},  # unit is degress
        {'angle_crop_max': 225.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 8.0}   # unit is meter
      ]
  )

  # Off by default: robot_state_publisher already publishes base_link ->
  # base_scan from robot_esp.urdf (0.15 m up). Turn it on only to view the
  # LiDAR on its own, without the robot bringup, or the two fight.
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_scan_ld14p',
    condition=IfCondition(publish_static_tf),
    arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', frame_id]
  )

  ld = LaunchDescription()

  ld.add_action(DeclareLaunchArgument(
    'bridge_ip', default_value='192.168.1.50',
    description='ESP32 bridge address in tcp_client mode; local bind address '
                'in tcp_server mode. Must match WIFI_STATIC_IP in the sketch.'))
  ld.add_action(DeclareLaunchArgument(
    'bridge_port', default_value='8889',
    description='TCP port of the bridge; must match BRIDGE_PORT in the sketch.'))
  ld.add_action(DeclareLaunchArgument(
    'comm_mode', default_value='tcp_client',
    description='serial, tcp_client, tcp_server, udp_client or udp_server.'))
  ld.add_action(DeclareLaunchArgument(
    'product_name', default_value='LDLiDAR_LD14P',
    description='LDLiDAR_LD14P or LDLiDAR_LD14 - it selects the parser '
                'profile, so it must match the unit, not the bridge.'))
  ld.add_action(DeclareLaunchArgument(
    'frame_id', default_value='base_scan',
    description='base_scan matches robot_esp.urdf; the stock launch files use '
                'base_laser, which the URDF does not define.'))
  ld.add_action(DeclareLaunchArgument(
    'topic_name', default_value='scan',
    description='Remap this if something else is already publishing /scan.'))
  ld.add_action(DeclareLaunchArgument(
    'publish_static_tf', default_value='false',
    description='Publish base_link -> frame_id here. Leave false when '
                'robot_state_publisher is running.'))

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
