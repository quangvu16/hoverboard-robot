"""Copy the shared /tf and /tf_static into this node's namespace.

Nav2's bringup, when namespaced, listens on <ns>/tf rather than /tf. The
hoverboard robots all publish ONE tree on /tf with prefixed frames
(robot1/odom, robot2/base_link, ...) - which is what lets the formation nodes
relate one robot to another - so each namespaced Nav2 gets a relayed copy.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = Node('tf_relay')
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(TFMessage, 'tf', 100)
    pub_static = node.create_publisher(TFMessage, 'tf_static', static_qos)
    node.create_subscription(TFMessage, '/tf', pub.publish, 100)
    node.create_subscription(TFMessage, '/tf_static', pub_static.publish, static_qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
