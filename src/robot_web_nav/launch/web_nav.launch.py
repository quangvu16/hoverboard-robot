"""Navigation from a web page over a websocket.

Starts rosbridge (websocket), a static web server for the page, and
web_nav_bridge; optionally also the navigation stack. Run robot_bringup first
(or let robot-bringup.service do it).

  ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream
  ros2 launch robot_web_nav web_nav.launch.py stack:=amcl map:=$HOME/m_map.yaml
  ros2 launch robot_web_nav web_nav.launch.py stack:=none      # nav already running

then open http://<robot-pc>:8000 in a browser on the same network.

Arguments:
  stack:=cartographer|amcl|none
      cartographer  robot_cartographer cartographer_nav.launch.py: map:=...pbstream
                    = localization on a saved map, no map = map while navigating
      amcl          robot_navigation navigation2.launch.py (Nav2 + AMCL), map:=...yaml
      none          only the web parts; start the nav stack yourself
  map:=PATH         passed to the chosen stack ('' = that launch's default)
  wheel_odom:=true|false  cartographer only; must match the bringup
  rviz:=false       cartographer only; the page replaces RViz (amcl's launch
                    always opens RViz)
  ws_port:=9090  http_port:=8000
  docking:=true|false  the dock manager (robot_docking): Set dock here / Dock /
                    Undock on the page, auto-dock when the battery is low
  auto_dock:=true|false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nav_stack(context):
    stack = LaunchConfiguration('stack').perform(context)
    map_path = os.path.expanduser(LaunchConfiguration('map').perform(context))
    if stack == 'cartographer':
        args = {'wheel_odom': LaunchConfiguration('wheel_odom').perform(context),
                'rviz': LaunchConfiguration('rviz').perform(context)}
        if map_path:
            args['map'] = map_path
        launch_file = os.path.join(get_package_share_directory('robot_cartographer'),
                                   'launch', 'cartographer_nav.launch.py')
    elif stack == 'amcl':
        args = {'map': map_path} if map_path else {}
        launch_file = os.path.join(get_package_share_directory('robot_navigation'),
                                   'launch', 'navigation2.launch.py')
    else:
        return [LogInfo(msg='stack:=none - start Cartographer or Nav2 yourself')]
    if map_path and not os.path.isfile(map_path):
        return [LogInfo(msg=f'map file not found: {map_path} - nav stack NOT started')]
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file),
                                     launch_arguments=args.items())]


def generate_launch_description():
    share = get_package_share_directory('robot_web_nav')
    rosbridge = os.path.join(get_package_share_directory('rosbridge_server'),
                             'launch', 'rosbridge_websocket_launch.xml')

    return LaunchDescription([
        DeclareLaunchArgument('stack', default_value='cartographer',
                              choices=['cartographer', 'amcl', 'none']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('wheel_odom', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('docking', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('auto_dock', default_value='true', choices=['true', 'false']),

        IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge),
                                 launch_arguments={'port': LaunchConfiguration('ws_port')}.items()),
        Node(package='robot_web_nav', executable='web_nav_bridge', output='screen',
             parameters=[{'stack': LaunchConfiguration('stack')}]),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', LaunchConfiguration('http_port'),
                 '--directory', os.path.join(share, 'web')],
            output='log', name='web_page'),
        LogInfo(msg=['Web navigation page: http://<this-pc>:', LaunchConfiguration('http_port')]),
        OpaqueFunction(function=nav_stack),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_docking'), 'launch', 'docking.launch.py')),
            launch_arguments={'auto_dock': LaunchConfiguration('auto_dock')}.items(),
            condition=IfCondition(LaunchConfiguration('docking'))),
    ])
