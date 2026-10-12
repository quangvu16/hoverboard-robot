"""Pass goals from the Yahboom app on to Nav2.

The app publishes geometry_msgs/PoseStamped on /goal_pose with an empty
frame_id; Nav2 needs "map". This republishes such goals once, with the frame
filled in. Goals that already carry a frame are left alone - which is also
what stops this node re-reading its own output (Yahboom's version republished
every message it received on the same topic, looping forever).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class AppSendGoal(Node):
    def __init__(self):
        super().__init__('app_send_goal')
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 1)

    def on_goal(self, msg):
        if msg.header.frame_id:
            return
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(
            f'goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) -> {self.frame}')


def main():
    rclpy.init()
    node = AppSendGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
