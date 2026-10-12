"""Shared driver for the odometry calibration nodes (Yahboom calibrate_*, for hoverboards)."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CalibrationNode(Node):
    """Drive a test by odometry when start_test is set true; then compare with
    a measured value (measured_* parameter) and print the corrected geometry."""

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('start_test', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 5)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.running = False
        self.create_timer(0.05, self.on_timer)
        self.add_on_set_parameters_callback(self.on_params)

    def pose(self):
        t = self.buf.lookup_transform(self.get_parameter('odom_frame').value,
                                      self.get_parameter('base_frame').value, rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def finish(self):
        self.cmd_vel.publish(Twist())
        self.running = False
        self.set_parameters([rclpy.parameter.Parameter('start_test', value=False)])

    def on_timer(self):
        if not self.get_parameter('start_test').value:
            if self.running:
                self.finish()
            return
        try:
            pose = self.pose()
        except Exception as e:
            self.get_logger().warn(f'no odometry TF yet: {e}', throttle_duration_sec=2.0)
            self.cmd_vel.publish(Twist())
            return
        if not self.running:
            self.running = True
            self.begin(pose)
        cmd = self.step(pose)
        if cmd is None:
            self.finish()
        else:
            self.cmd_vel.publish(cmd)

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name.startswith('measured_') and p.value and p.value > 0:
                self.report(p.value)
        return SetParametersResult(successful=True)
