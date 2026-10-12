"""Wander and avoid obstacles with the LiDAR (Yahboom laser_Avoidance, for the hoverboard).

    ros2 run hoverboard_laser laser_Avoidance
    ros2 param set /laser_Avoidance Switch true      # pause

Three sectors: front |a| <= 20 deg, left 20..LaserAngle, right -LaserAngle..-20.
A sector is "blocked" when more than min_points returns lie inside
ResponseDist * 1.5. Each decision is held for a short time (Yahboom slept
inside the scan callback to do that; here a timer holds it). Paused while
JoyState is true (gamepad in manual) or Switch is set.

Parameters (live-tunable): linear 0.2 m/s, angular 0.8 rad/s, LaserAngle 45,
ResponseDist 0.55 m, min_points 5, min_range 0.30 m, Switch false.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from hoverboard_laser.common import scan_points, twist


class LaserAvoid(Node):
    def __init__(self):
        super().__init__('laser_Avoidance')
        for name, default in [('linear', 0.2), ('angular', 0.8), ('LaserAngle', 45.0),
                              ('ResponseDist', 0.55), ('min_points', 5), ('min_range', 0.30),
                              ('Switch', False)]:
            self.declare_parameter(name, default)
        self.joy_active = False
        self.cmd = Twist()
        self.hold_until = 0.0
        self.moving = False
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/JoyState', lambda m: setattr(self, 'joy_active', m.data), 10)
        self.create_timer(0.1, self.on_timer)

    def p(self, name):
        return self.get_parameter(name).value

    def paused(self):
        return self.joy_active or self.p('Switch')

    def on_scan(self, scan):
        if self.paused() or time.monotonic() < self.hold_until:
            return
        ang, r = scan_points(scan, self.p('min_range'))
        near = r < self.p('ResponseDist') * 1.5
        a = self.p('LaserAngle')
        n = self.p('min_points')
        front = int(((abs(ang) <= 20) & near).sum()) > n
        left = int(((ang > 20) & (ang < a) & near).sum()) > n
        right = int(((ang < -20) & (ang > -a) & near).sum()) > n
        v, w = self.p('linear'), self.p('angular')

        # Same decision table as Yahboom's, with a hold time per manoeuvre.
        if front and left and right:
            cmd, hold, why = twist(0.0, -w), 0.4, 'boxed in: turn right on the spot'
        elif front and right:
            cmd, hold, why = twist(v * 0.5, w), 0.2, 'front+right blocked: turn left'
        elif front and left:
            cmd, hold, why = twist(v * 0.5, -w), 0.2, 'front+left blocked: turn right'
        elif front:
            cmd, hold, why = twist(0.0, w), 0.2, 'front blocked: turn left'
        elif left and right:
            cmd, hold, why = twist(v * 0.5, -w), 0.4, 'narrow gap: turn right'
        elif left:
            cmd, hold, why = twist(v, -w), 0.2, 'left blocked: veer right'
        elif right:
            cmd, hold, why = twist(v, w), 0.2, 'right blocked: veer left'
        else:
            cmd, hold, why = twist(v, 0.0), 0.0, 'clear: forward'
        # A blocked front never drives forward into it (Yahboom kept linear>0).
        self.cmd = cmd
        self.hold_until = time.monotonic() + hold
        self.get_logger().info(why, throttle_duration_sec=1.0)

    def on_timer(self):
        if self.paused():
            if self.moving:
                self.pub_vel.publish(Twist())
                self.moving = False
            return
        self.moving = True
        self.pub_vel.publish(self.cmd)


def main():
    rclpy.init()
    node = LaserAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub_vel.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
