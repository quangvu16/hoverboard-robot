"""How much of the map has the robot covered?

ROS 2 port of nobleo full_coverage_path_planner's coverage_progress (its
ros2-branch copy was still half rospy and could not run). One change: coverage
is measured against the FREE cells of /map instead of a hand-entered
rectangle, so the original's coverage_area_offset / size arguments are gone.

Publishes
    coverage_progress  std_msgs/Float32   0..1, covered free cells / all free cells
    coverage_grid      nav_msgs/OccupancyGrid  0 = covered, 100 = not yet
Service
    ~/reset            std_srvs/Trigger
Parameters: coverage_radius 0.165 m (half the robot width - the "tool"),
map_frame map, coverage_frame base_link, rate 2 Hz.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class CoverageProgress(Node):
    def __init__(self):
        super().__init__('coverage_progress')
        self.radius = float(self.declare_parameter('coverage_radius', 0.165).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.cov_frame = self.declare_parameter('coverage_frame', 'base_link').value
        rate = float(self.declare_parameter('rate', 2.0).value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.pub_progress = self.create_publisher(Float32, 'coverage_progress', 1)
        self.pub_grid = self.create_publisher(OccupancyGrid, 'coverage_grid', 1)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.map = None
        self.covered = None
        self.create_timer(1.0 / rate, self.update)

    def on_map(self, m):
        first = self.map is None or (m.info.width, m.info.height) != (self.map.info.width, self.map.info.height)
        self.map = m
        data = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        self.free = (data >= 0) & (data < 50)
        if first:
            self.covered = np.zeros_like(self.free)
            r = self.radius / m.info.resolution
            n = int(np.ceil(r))
            yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
            self.disk = (xx ** 2 + yy ** 2) <= r ** 2
            self.get_logger().info(f'map {m.info.width}x{m.info.height} @ {m.info.resolution} m, '
                                   f'{int(self.free.sum())} free cells to cover')

    def on_reset(self, _req, res):
        if self.covered is not None:
            self.covered[:] = False
        res.success, res.message = True, 'coverage reset'
        return res

    def update(self):
        if self.map is None:
            return
        try:
            tf = self.buf.lookup_transform(self.map_frame, self.cov_frame, rclpy.time.Time())
        except Exception:
            return
        info = self.map.info
        cx = int((tf.transform.translation.x - info.origin.position.x) / info.resolution)
        cy = int((tf.transform.translation.y - info.origin.position.y) / info.resolution)
        n = self.disk.shape[0] // 2
        y0, y1 = max(cy - n, 0), min(cy + n + 1, info.height)
        x0, x1 = max(cx - n, 0), min(cx + n + 1, info.width)
        if y0 < y1 and x0 < x1:
            d = self.disk[y0 - (cy - n):y1 - (cy - n), x0 - (cx - n):x1 - (cx - n)]
            self.covered[y0:y1, x0:x1] |= d
        total = int(self.free.sum())
        done = int((self.covered & self.free).sum())
        self.pub_progress.publish(Float32(data=done / total if total else 0.0))

        g = OccupancyGrid()
        g.header.frame_id = self.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.info = info
        grid = np.full(self.free.shape, -1, dtype=np.int8)
        grid[self.free] = 100
        grid[self.covered & self.free] = 0
        g.data = grid.ravel().tolist()
        self.pub_grid.publish(g)
        self.get_logger().info(f'coverage {100.0 * done / max(total, 1):.1f} %', throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = CoverageProgress()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
