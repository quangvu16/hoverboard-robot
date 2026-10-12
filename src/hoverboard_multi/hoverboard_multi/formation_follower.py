"""Drive one follower to its formation slot (Yahboom's "listenline", for hoverboards).

Yahboom's follower was written for a mecanum car and steered with linear.y,
which a differential-drive hoverboard cannot do. This one turns towards the
slot, drives to it, then matches the slot's heading.

    ros2 run hoverboard_multi formation_follower --ros-args \\
        -p robot_frame:=robot2/base_footprint -p target_frame:=point1 \\
        -p cmd_vel_topic:=/robot2/cmd_vel

Needs a TF path between the two robots (both localized in the shared "map"
frame, e.g. hoverboard_multi robot_nav_multi.launch.py for each). Paused while
/JoyState is true or Switch is false. Stops if the TF goes missing.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FormationFollower(Node):
    def __init__(self):
        super().__init__('formation_follower')
        p = self.declare_parameter
        self.robot_frame = p('robot_frame', 'robot2/base_footprint').value
        self.target_frame = p('target_frame', 'point1').value
        topic = p('cmd_vel_topic', '/robot2/cmd_vel').value
        p('Switch', True)
        self.max_v = float(p('max_linear', 0.25).value)
        self.max_w = float(p('max_angular', 0.8).value)
        self.pos_tol = float(p('position_tolerance', 0.10).value)
        self.yaw_tol = float(p('heading_tolerance', 0.15).value)
        self.joy = False
        self.pub = self.create_publisher(Twist, topic, 5)
        self.create_subscription(Bool, '/JoyState', lambda m: setattr(self, 'joy', m.data), 1)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_timer(0.1, self.step)
        self.was_moving = False

    def stop(self):
        if self.was_moving:
            self.pub.publish(Twist())
            self.was_moving = False

    def step(self):
        if self.joy or not self.get_parameter('Switch').value:
            self.stop()
            return
        try:
            # Where the slot is, seen from the follower.
            tf = self.buf.lookup_transform(self.robot_frame, self.target_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f'waiting for TF {self.robot_frame} -> {self.target_frame}: {e}',
                                   throttle_duration_sec=2.0)
            self.stop()
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        dist = math.hypot(x, y)
        bearing = math.atan2(y, x)
        cmd = Twist()
        if dist > self.pos_tol:
            # Face the slot first, then drive; reverse briefly if it is just behind.
            if abs(bearing) > math.radians(100) and dist < 0.4:
                cmd.linear.x = -min(self.max_v, 0.8 * dist)
            elif abs(bearing) > math.radians(30):
                cmd.angular.z = math.copysign(min(self.max_w, 1.2 * abs(bearing)), bearing)
            else:
                cmd.linear.x = min(self.max_v, 0.8 * dist) * math.cos(bearing)
                cmd.angular.z = max(-self.max_w, min(self.max_w, 1.5 * bearing))
        else:
            yaw_err = yaw_of(tf.transform.rotation)
            if abs(yaw_err) > self.yaw_tol:
                cmd.angular.z = math.copysign(min(self.max_w, 1.0 * abs(yaw_err)), yaw_err)
        self.pub.publish(cmd)
        self.was_moving = True


def main():
    rclpy.init()
    node = FormationFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()
