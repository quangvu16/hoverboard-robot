"""Turn to face the nearest obstacle ahead and sound the buzzer when it is close
(Yahboom laser_Warning, for the hoverboard).

    ros2 run hoverboard_laser laser_Warning

Buzzer: std_msgs/Bool on /Buzzer, published only when the state changes
(Yahboom sent a UInt16 on /beep on every scan). hoverboard_udp_bridge_node
forwards it to the ESP32. Paused while JoyState is true or Switch is set.

Parameters: LaserAngle 10, ResponseDist 0.5 m (Yahboom's 0.3 is inside
the 0.30 m robot-body filter on this 0.33 m chassis), max_angular 0.8 rad/s,
turn_to_face true, min_range 0.30 m, Switch false.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from hoverboard_laser.common import SinglePID, scan_points, twist


class LaserWarning(Node):
    def __init__(self):
        super().__init__('laser_Warning')
        for name, default in [('LaserAngle', 10.0), ('ResponseDist', 0.5), ('max_angular', 0.8),
                              ('turn_to_face', True), ('min_range', 0.30), ('Switch', False)]:
            self.declare_parameter(name, default)
        self.joy_active = False
        self.moving = False
        self.buzzer = None
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_buzzer = self.create_publisher(Bool, '/Buzzer', 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/JoyState', lambda m: setattr(self, 'joy_active', m.data), 10)

    def p(self, name):
        return self.get_parameter(name).value

    def set_buzzer(self, on):
        if on != self.buzzer:
            self.pub_buzzer.publish(Bool(data=on))
            self.buzzer = on

    def on_scan(self, scan):
        if self.joy_active or self.p('Switch'):
            if self.moving:
                self.pub_vel.publish(Twist())
                self.moving = False
            self.set_buzzer(False)
            return
        ang, r = scan_points(scan, self.p('min_range'))
        window = abs(ang) < self.p('LaserAngle')
        if not window.any():
            self.set_buzzer(False)
            return
        i = np.argmin(np.where(window, r, np.inf))
        min_dist, min_ang = float(r[i]), float(ang[i])
        if self.p('turn_to_face'):
            self.moving = True
            self.ang_pid.limit = self.p('max_angular')
            w = self.ang_pid.pid_compute(min_ang / 48.0, 0.0)
            self.pub_vel.publish(twist(0.0, 0.0 if abs(w) < 0.1 else w))
        self.set_buzzer(min_dist <= self.p('ResponseDist'))


def main():
    rclpy.init()
    node = LaserWarning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub_vel.publish(Twist())
    node.pub_buzzer.publish(Bool(data=False))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
