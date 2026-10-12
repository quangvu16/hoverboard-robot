"""In-place rotation odometry calibration -> wheel_separation_m.

    ros2 run hoverboard_camera calibrate_angular
    ros2 param set /calibrate_angular start_test true       # turns test_angle
    # measure how far the robot REALLY turned (degrees), then:
    ros2 param set /calibrate_angular measured_angle 330.0

Prints the corrected wheel_separation_m for
hoverboard_base_node/config/hoverboard_base.yaml. If odometry reports more
turn than really happened, the configured separation is too small.
Alternates direction each run. (With the LiDAR mounted,
~/robot_ws/tools/calib_wheel_separation.py measures this without a protractor.)
Parameters: test_angle 360 deg, speed 0.5 rad/s, tolerance 1.5 deg,
wheel_separation_m 0.18 (the value the bridge is running with).
"""
import math

import rclpy
from geometry_msgs.msg import Twist

from hoverboard_camera.calibrate_common import CalibrationNode


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CalibrateAngular(CalibrationNode):
    def __init__(self):
        super().__init__('calibrate_angular')
        self.declare_parameter('test_angle', 360.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('tolerance', 1.5)
        self.declare_parameter('wheel_separation_m', 0.18)
        self.declare_parameter('measured_angle', 0.0)
        self.direction = 1.0
        self.last_odom = None

    def begin(self, pose):
        self.prev_yaw = pose[2]          # the robot's real heading, not 0
        self.turned = 0.0
        self.get_logger().info(f"turning {'left' if self.direction > 0 else 'right'} "
                               f"{self.get_parameter('test_angle').value} deg by odometry")

    def step(self, pose):
        self.turned += wrap(pose[2] - self.prev_yaw)
        self.prev_yaw = pose[2]
        target = math.radians(self.get_parameter('test_angle').value)
        remaining = target - abs(self.turned)
        if remaining <= math.radians(self.get_parameter('tolerance').value):
            self.last_odom = math.degrees(abs(self.turned))
            self.direction = -self.direction
            self.get_logger().info(f'done: odometry says {self.last_odom:.1f} deg. Measure the real turn '
                                   f'and set measured_angle to get the corrected wheel_separation_m.')
            return None
        cmd = Twist()
        cmd.angular.z = self.direction * max(0.2, min(self.get_parameter('speed').value, 1.5 * remaining))
        return cmd

    def report(self, measured):
        if not self.last_odom:
            self.get_logger().warn('run a test first (start_test true)')
            return
        s = self.get_parameter('wheel_separation_m').value
        self.get_logger().info(f'odometry {self.last_odom:.1f} deg vs measured {measured:.1f} deg -> '
                               f'wheel_separation_m: {s * self.last_odom / measured:.4f} (was {s})')


def main():
    rclpy.init()
    node = CalibrateAngular()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_vel.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()
