#!/usr/bin/env python3
"""nav_msgs/Odometry from Cartographer's odom -> base_footprint TF.

Without wheel encoders (cartographer_nav.launch.py wheel_odom:=false)
Cartographer is the odometry, but it only publishes TF. Nav2's controller
(MPPI) needs the robot's current velocity on an Odometry topic, so this node
differentiates the TF and publishes it on odom/cartographer (the launch file
points Nav2's odom_topic there).

Parameters: odom_frame (odom), base_frame (base_footprint), rate_hz (20),
smoothing (0.5: weight of the newest velocity sample, 1 = none).
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CartographerOdom(Node):

    def __init__(self):
        super().__init__('cartographer_odom')
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('rate_hz', 20.0).value
        self.alpha = self.declare_parameter('smoothing', 0.5).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, 'odom/cartographer', 10)
        self.prev = None          # (stamp_s, x, y, yaw)
        self.vx = self.vy = self.wz = 0.0
        self.warned = False
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time(), Duration(seconds=0.0))
        except TransformException as e:
            if not self.warned:
                self.get_logger().warn(f'waiting for {self.odom_frame} -> {self.base_frame}: {e}')
                self.warned = True
            return

        stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        p, q = t.transform.translation, t.transform.rotation
        yaw = yaw_of(q)

        if self.prev is not None:
            dt = stamp - self.prev[0]
            if dt <= 0.0:
                return          # no new pose since the last tick
            dx, dy = p.x - self.prev[1], p.y - self.prev[2]
            dyaw = math.atan2(math.sin(yaw - self.prev[3]), math.cos(yaw - self.prev[3]))
            # Velocity in the robot frame (REP-105 twist is in child_frame_id).
            c, s = math.cos(yaw), math.sin(yaw)
            a = self.alpha
            self.vx = a * ((c * dx + s * dy) / dt) + (1 - a) * self.vx
            self.vy = a * ((-s * dx + c * dy) / dt) + (1 - a) * self.vy
            self.wz = a * (dyaw / dt) + (1 - a) * self.wz
        self.prev = (stamp, p.x, p.y, yaw)

        o = Odometry()
        o.header.stamp = t.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = p.x, p.y
        o.pose.pose.orientation = q
        o.twist.twist.linear.x = self.vx
        o.twist.twist.linear.y = self.vy
        o.twist.twist.angular.z = self.wz
        for i, v in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.02)):
            o.pose.covariance[i] = v
            o.twist.covariance[i] = v
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(CartographerOdom())


if __name__ == '__main__':
    main()
