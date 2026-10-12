#!/usr/bin/env python3
"""
load_slam_map.py  –  call /slam_toolbox/deserialize_map once

Parameters
----------
map_file  : absolute path to *.posegraph*
start_x   : [m]
start_y   : [m]
start_yaw : [rad] (map frame, CCW+)
delay_sec : seconds to wait before calling the service (default 3)
"""
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph
from geometry_msgs.msg import Pose2D

class Loader(Node):
    def __init__(self):
        super().__init__('slam_map_loader')

        # ── declare params ─────────────────────────────────────────────
        self.declare_parameter('map_file', '/tmp/map.posegraph')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('delay_sec', 3.0)

        # ── build request ──────────────────────────────────────────────
        self.req = DeserializePoseGraph.Request()
        self.req.filename = self.get_parameter('map_file').value
        self.req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        self.req.initial_pose = Pose2D(
            x=float(self.get_parameter('start_x').value),
            y=float(self.get_parameter('start_y').value),
            theta=float(self.get_parameter('start_yaw').value))

        # ── client & delayed call ──────────────────────────────────────
        self.cli = self.create_client(DeserializePoseGraph,
                                      '/slam_toolbox/deserialize_map')
        delay = float(self.get_parameter('delay_sec').value)
        self.create_timer(delay, self._send_request)   # one-shot
        self.future = None

    # ------------------------------------------------------------------
    def _send_request(self):
        if not self.cli.service_is_ready():
            self.get_logger().error('deserialize_map service not ready')
            return
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(
            f'request sent for {self.req.filename} at '
            f'({self.req.initial_pose.x:.2f},'
            f'{self.req.initial_pose.y:.2f},'
            f'{self.req.initial_pose.theta:.2f})')

# ══════════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = Loader()

    # spin until `future` appears
    while rclpy.ok() and node.future is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    # spin until service completes
    if rclpy.ok():
        rclpy.spin_until_future_complete(node, node.future)

    # results
    if node.future and node.future.result() is not None:
        node.get_logger().info('✅ Pose-graph loaded successfully')
    else:
        reason = (node.future.exception() if node.future else 'future not set')
        node.get_logger().error(f'❌ Failed to deserialize map: {reason}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
