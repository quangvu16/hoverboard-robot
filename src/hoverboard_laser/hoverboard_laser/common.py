"""Shared helpers for the hoverboard LiDAR behaviours."""
import math

import numpy as np
from geometry_msgs.msg import Twist


class SinglePID:
    """Yahboom's PID (per-sample, no dt), with an output clamp added."""

    def __init__(self, P=0.1, I=0.0, D=0.1, limit=float('inf')):
        self.Kp, self.Ki, self.Kd, self.limit = P, I, D, limit
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        self.prevError = self.error
        out = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        return max(-self.limit, min(self.limit, out))

    def pid_reset(self):
        self.error = self.intergral = self.derivative = self.prevError = 0.0


# Yaw of base_scan in base_link: keep in step with scan_joint in
# hoverboard_description/urdf/hoverboard.urdf (LiDAR 0 deg faces forward).
LASER_YAW_DEG = 0.0


def scan_points(scan, min_range, yaw_deg=LASER_YAW_DEG):
    """Valid returns as (angle_deg, range_m) arrays.

    angle_deg is in the ROBOT frame: the scan angle rotated by yaw_deg about
    Z, wrapped to (-180, 180], 0 = straight ahead, positive = to the LEFT
    (CCW). Wrapping BOTH ways matters here:
    udp_lidar_receiver_node publishes angle_min 0 with a NEGATIVE increment,
    i.e. angles 0 .. -359 deg; Yahboom's code only wrapped values above 180,
    so everything front-left (+1..+45) came through as -359..-315 and was
    never looked at.

    Returns closer than min_range are dropped: the robot's own chassis sits
    inside ~0.3 m of the LiDAR.
    """
    r = np.asarray(scan.ranges, dtype=float)
    ang = np.degrees(scan.angle_min + scan.angle_increment * np.arange(len(r)))
    ang = (ang + yaw_deg + 180.0) % 360.0 - 180.0
    ok = np.isfinite(r) & (r >= max(min_range, scan.range_min)) & (r <= scan.range_max)
    return ang[ok], r[ok]


def twist(v=0.0, w=0.0):
    t = Twist()
    t.linear.x = float(v)
    t.angular.z = float(w)
    return t
