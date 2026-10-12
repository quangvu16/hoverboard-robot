"""Straight-line odometry calibration -> wheel_radius_m.

    ros2 run hoverboard_camera calibrate_linear
    ros2 param set /calibrate_linear start_test true        # drives test_distance
    # measure how far the robot REALLY went, then:
    ros2 param set /calibrate_linear measured_distance 0.97

Prints the corrected wheel_radius_m for
hoverboard_base_node/config/hoverboard_base.yaml. Runs alternate forwards and
backwards so the robot ends where it started. Differential drive only (Yahboom's
sideways option was for a mecanum car).
Parameters: test_distance 1.0 m, speed 0.2 m/s, tolerance 0.02 m,
wheel_radius_m 0.09615 (the value the bridge is running with).
"""
import math

import rclpy
from geometry_msgs.msg import Twist

from hoverboard_camera.calibrate_common import CalibrationNode


class CalibrateLinear(CalibrationNode):
    def __init__(self):
        super().__init__('calibrate_linear')
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('tolerance', 0.02)
        self.declare_parameter('wheel_radius_m', 0.09615)
        self.declare_parameter('measured_distance', 0.0)
        self.direction = 1.0
        self.last_odom = None

    def begin(self, pose):
        self.start = pose
        self.get_logger().info(f"driving {'forward' if self.direction > 0 else 'backward'} "
                               f"{self.get_parameter('test_distance').value} m by odometry")

    def step(self, pose):
        target = self.get_parameter('test_distance').value
        d = math.hypot(pose[0] - self.start[0], pose[1] - self.start[1])
        if d >= target - self.get_parameter('tolerance').value:
            self.last_odom = d
            self.direction = -self.direction
            self.get_logger().info(f'done: odometry says {d:.3f} m. Measure the real distance and set '
                                   f'measured_distance to get the corrected wheel_radius_m.')
            return None
        cmd = Twist()
        # Slow down over the last 10 cm so coasting does not overshoot.
        cmd.linear.x = self.direction * max(0.05, min(self.get_parameter('speed').value, 1.5 * (target - d)))
        return cmd

    def report(self, measured):
        if not self.last_odom:
            self.get_logger().warn('run a test first (start_test true)')
            return
        r = self.get_parameter('wheel_radius_m').value
        self.get_logger().info(f'odometry {self.last_odom:.3f} m vs measured {measured:.3f} m -> '
                               f'wheel_radius_m: {r * measured / self.last_odom:.5f} (was {r})')


def main():
    rclpy.init()
    node = CalibrateLinear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_vel.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()
