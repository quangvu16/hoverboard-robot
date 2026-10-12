"""Ask the app_save_map server to save the current map.

    ros2 run hoverboard_app_save_map client <mapname> [service_name]
"""
import sys

import rclpy
from rclpy.node import Node

from hoverboard_web_savmap_interfaces.srv import WebSaveMap


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    if len(argv) < 2:
        print('usage: ros2 run hoverboard_app_save_map client <mapname> [service_name]')
        return
    name = argv[1]
    service = argv[2] if len(argv) > 2 else 'yahboomAppSaveMap'

    node = Node('app_save_map_client')
    cli = node.create_client(WebSaveMap, service)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'service "{service}" not available - is the server running?')
        return
    req = WebSaveMap.Request()
    req.mapname = name
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
    if future.result() is None:
        node.get_logger().error('no response')
    else:
        print(future.result().response)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
