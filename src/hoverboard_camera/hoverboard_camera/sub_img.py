"""Show the camera stream in a window, with the real frame rate.

    ros2 run hoverboard_camera sub_img
    ros2 run hoverboard_camera sub_img --ros-args -p topic:=/camera/image_raw/compressed

(Yahboom's version also re-published the frames; camera_source already
publishes /image_raw, so this one only displays.)
"""
import time

import cv2 as cv
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class SubImg(Node):
    def __init__(self):
        super().__init__('sub_img')
        self.bridge = CvBridge()
        topic = self.declare_parameter('topic', '/camera/image_raw/compressed').value
        self.create_subscription(CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        self.t_prev = None
        self.fps = 0.0

    def on_image(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        now = time.monotonic()
        if self.t_prev is not None:
            self.fps = 0.9 * self.fps + 0.1 / max(now - self.t_prev, 1e-3)
        self.t_prev = now
        cv.putText(frame, f'FPS: {self.fps:.1f}', (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.imshow('camera', frame)
        cv.waitKey(1)


def main():
    rclpy.init()
    node = SubImg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()
