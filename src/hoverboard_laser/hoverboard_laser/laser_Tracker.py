"""Follow the nearest object ahead, keeping ResponseDist to it (Yahboom laser_Tracker).

    ros2 run hoverboard_laser laser_Tracker

Targets within priorityAngle of straight ahead win; otherwise the nearest
return within LaserAngle. Drives to hold ResponseDist and turns to face it.
Stops if nothing is in view (Yahboom crashed on min() of an empty list).
Paused while JoyState is true or Switch is set.

Parameters: priorityAngle 10, LaserAngle 30, ResponseDist 0.55 m,
max_linear 0.25 m/s, max_angular 0.8 rad/s, min_range 0.30 m, Switch false.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from hoverboard_laser.common import SinglePID, scan_points, twist


class LaserTracker(Node):
    def __init__(self):
        super().__init__('laser_Tracker')
        for name, default in [('priorityAngle', 10.0), ('LaserAngle', 30.0), ('ResponseDist', 0.55),
                              ('max_linear', 0.25), ('max_angular', 0.8), ('min_range', 0.30),
                              ('Switch', False)]:
            self.declare_parameter(name, default)
        self.joy_active = False
        self.moving = False
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/JoyState', lambda m: setattr(self, 'joy_active', m.data), 10)

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, scan):
        if self.joy_active or self.p('Switch'):
            if self.moving:
                self.pub_vel.publish(Twist())
                self.moving = False
            return
        self.moving = True
        ang, r = scan_points(scan, self.p('min_range'))
        dist = self.p('ResponseDist')
        front = (abs(ang) < self.p('priorityAngle')) & (r < dist + 0.5)
        window = abs(ang) < self.p('LaserAngle')
        pick = front if front.any() else window
        if not pick.any():
            self.pub_vel.publish(Twist())      # nothing to follow
            return
        i = np.argmin(np.where(pick, r, np.inf))
        min_dist, min_ang = float(r[i]), float(ang[i])
        if abs(min_dist - dist) < 0.1:
            min_dist = dist
        self.lin_pid.limit = self.p('max_linear')
        self.ang_pid.limit = self.p('max_angular')
        v = -self.lin_pid.pid_compute(dist, min_dist)
        w = self.ang_pid.pid_compute(min_ang / 48.0, 0.0)
        # ang_pid(target, current) = Kp*(target - 0): positive when the object is
        # to the left, so w > 0 turns towards it.
        if abs(w) < 0.1:
            w = 0.0
        self.pub_vel.publish(twist(v, w))


def main():
    rclpy.init()
    node = LaserTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub_vel.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
