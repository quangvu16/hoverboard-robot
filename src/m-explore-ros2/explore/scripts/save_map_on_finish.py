#!/usr/bin/env python3
"""Save the map once explore_lite reports it has finished (hoverboard addition).

Waits for std_msgs/Bool true on explore/finished (latched), then runs
nav2_map_server's map_saver_cli into map_path (.yaml/.pgm).
"""
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class SaveOnFinish(Node):
    def __init__(self):
        super().__init__('save_map_on_finish')
        self.path = self.declare_parameter('map_path', '').value
        self.done = False
        self.create_subscription(Bool, 'explore/finished', self.on_finished,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info(f'will save the map to {self.path}.yaml when exploration finishes')

    def on_finished(self, msg):
        if not msg.data or self.done:
            return
        self.done = True
        self.get_logger().info('exploration finished, saving map...')
        r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.path,
                            '--ros-args', '-p', 'save_map_timeout:=10000.0'],
                           capture_output=True, text=True, timeout=60)
        if r.returncode == 0:
            self.get_logger().info(f'map saved: {self.path}.yaml')
        else:
            self.get_logger().error('map_saver_cli failed: ' + (r.stderr or r.stdout)[-300:])


def main():
    rclpy.init()
    node = SaveOnFinish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
