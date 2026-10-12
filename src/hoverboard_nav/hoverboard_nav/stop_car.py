"""Keep publishing nothing, and send a stop on exit (Ctrl-C).

    ros2 run hoverboard_nav stop_car
    ros2 run hoverboard_nav stop_car --ros-args -p cmd_vel_topic:=/robot1/cmd_vel

Yahboom shelled out to `ros2 topic pub --once` for this; a publisher does it
directly. stop_robot1_car / stop_robot2_car are the namespaced shortcuts.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def run(default_topic):
    rclpy.init()
    node = Node('stop_car')
    topic = node.declare_parameter('cmd_vel_topic', default_topic).value
    pub = node.create_publisher(Twist, topic, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    for _ in range(5):
        pub.publish(Twist())
    node.get_logger().info(f'stop sent on {topic}')
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    run('/cmd_vel')


def main_robot1():
    run('/robot1/cmd_vel')


def main_robot2():
    run('/robot2/cmd_vel')
