"""Camera for the hoverboard vision packages (stands in for Yahboom's ESP32 camera board).

Publishes
    /camera/image_raw/compressed  sensor_msgs/CompressedImage (JPEG) - what every
                                  hoverboard_ai_car / hoverboard_mediapipe node reads
    /image_raw                    sensor_msgs/Image (bgr8), if publish_raw is true -
                                  for the colour/line trackers that want raw frames

source: a device index ("0" = /dev/video0) or any URL OpenCV can open, e.g. an
ESP32-CAM's MJPEG stream "http://192.168.1.50:81/stream".

    ros2 run hoverboard_camera camera_source --ros-args -p source:=0
    ros2 run hoverboard_camera camera_source --ros-args -p source:=http://192.168.1.50:81/stream

Reconnects by itself if the camera drops (WiFi cameras do).
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class CameraSource(Node):
    def __init__(self):
        super().__init__('camera_source')
        self.source = str(self.declare_parameter('source', '0').value)
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        fps = float(self.declare_parameter('fps', 15.0).value)
        self.quality = int(self.declare_parameter('jpeg_quality', 80).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.publish_raw = bool(self.declare_parameter('publish_raw', True).value)
        self.bridge = CvBridge()
        self.pub_c = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_profile_sensor_data)
        self.pub_r = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data) if self.publish_raw else None
        self.cap = None
        self.last_open = 0.0
        self.frames = 0
        self.create_timer(1.0 / max(fps, 1.0), self.grab)
        self.create_timer(5.0, self.report)

    def open(self):
        if time.time() - self.last_open < 2.0:
            return False
        self.last_open = time.time()
        src = int(self.source) if self.source.isdigit() else self.source
        self.cap = cv2.VideoCapture(src)
        if isinstance(src, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        ok = self.cap.isOpened()
        self.get_logger().info(f'camera {self.source}: {"opened" if ok else "NOT available, retrying"}')
        return ok

    def grab(self):
        if self.cap is None or not self.cap.isOpened():
            if not self.open():
                return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.cap.release()
            self.cap = None
            return
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        stamp = self.get_clock().now().to_msg()
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = self.frame_id
            m.format = 'jpeg'
            m.data = jpg.tobytes()
            self.pub_c.publish(m)
        if self.pub_r is not None:
            r = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            r.header.stamp = stamp
            r.header.frame_id = self.frame_id
            self.pub_r.publish(r)
        self.frames += 1

    def report(self):
        if self.frames:
            self.get_logger().info(f'{self.frames / 5.0:.1f} fps from {self.source}')
        self.frames = 0


def main():
    rclpy.init()
    node = CameraSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.cap is not None:
        node.cap.release()
    node.destroy_node()
    rclpy.try_shutdown()
