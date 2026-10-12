"""Run one full-coverage pass: (optionally) localize, plan the coverage path once, follow it.

ROS 2 / Nav2 port of the globaledgesoft FCPP-on-TurtleBot3 flow, which in
ROS 1 was: spin 20 s at 2 rad/s so AMCL converges (intial_pose_estimation.py),
publish a dummy goal to trigger move_base_flex's SpiralSTC planner, then let
tracking_pid follow the path.

Here:
  1. optional localization spin (localize_spin), gentle enough for a hoverboard
  2. ComputePathToPose with planner_id SpiralSTC, ONCE - Nav2's NavigateToPose
     tree would replan every second and restart the coverage each time
  3. FollowPath with controller_id FollowCoverage (Regulated Pure Pursuit)

The plan is also published on /coverage_plan (latched) for RViz.

    ros2 run full_coverage_path_planning coverage_runner
    ros2 service call /coverage_runner/start std_srvs/srv/Trigger
    ros2 service call /coverage_runner/cancel std_srvs/srv/Trigger
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def densify(path, step):
    """Spiral-STC returns only lane corners, some repeated, with mixed headings.
    Pure pursuit then dithers at corners and the progress checker aborts. The
    original project put tracking_pid's path_interpolator in between for the
    same reason; this is its equivalent: drop repeats, put a pose every `step`
    metres along each lane, each facing along its lane."""
    pts = []
    for ps in path.poses:
        x, y = ps.pose.position.x, ps.pose.position.y
        if not pts or math.hypot(x - pts[-1][0], y - pts[-1][1]) > 1e-3:
            pts.append((x, y))
    out = Path()
    out.header = path.header
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(seg / step))
        for i in range(n):
            f = i / n
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x0 + f * (x1 - x0)
            ps.pose.position.y = y0 + f * (y1 - y0)
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            out.poses.append(ps)
    if len(pts) >= 2:
        last = PoseStamped()
        last.header = path.header
        last.pose.position.x, last.pose.position.y = pts[-1]
        last.pose.orientation = out.poses[-1].pose.orientation
        out.poses.append(last)
    return out


class CoverageRunner(Node):
    def __init__(self):
        super().__init__('coverage_runner')
        p = self.declare_parameter
        self.planner_id = p('planner_id', 'SpiralSTC').value
        self.controller_id = p('controller_id', 'FollowCoverage').value
        self.goal_checker_id = p('goal_checker_id', 'general_goal_checker').value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.auto_start = bool(p('auto_start', True).value)
        self.start_delay = float(p('start_delay', 5.0).value)
        # The original spun 20 s at 2 rad/s. For a heavy hoverboard base one slow
        # turn is enough for AMCL; 0 disables (e.g. already localized).
        self.spin_speed = float(p('localize_spin_speed', 0.5).value)
        self.spin_angle = math.radians(float(p('localize_spin_deg', 360.0).value))
        self.step = float(p('path_step', 0.05).value)

        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_plan = self.create_publisher(Path, 'coverage_plan', latched)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.follow_handle = None
        self.busy = False
        if self.auto_start:
            self.auto_timer = self.create_timer(self.start_delay, self.auto_once)
        self.get_logger().info(f'planner {self.planner_id}, controller {self.controller_id}; '
                               f'{"auto start in %.0f s" % self.start_delay if self.auto_start else "call ~/start"}')

    # ------------------------------------------------------------- services
    def auto_once(self):
        self.auto_timer.cancel()
        self.start()

    def on_start(self, _req, res):
        res.success = self.start()
        res.message = 'coverage started' if res.success else 'already running'
        return res

    def on_cancel(self, _req, res):
        if self.follow_handle is not None:
            self.follow_handle.cancel_goal_async()
        self.pub_cmd.publish(Twist())
        self.busy = False
        res.success, res.message = True, 'cancelled'
        return res

    # -------------------------------------------------------------- the run
    def start(self):
        if self.busy:
            return False
        self.busy = True
        if self.spin_speed > 0.0 and self.spin_angle > 0.0:
            self.localize_spin()
        else:
            self.request_plan()
        return True

    def localize_spin(self):
        duration = self.spin_angle / self.spin_speed
        self.get_logger().info(f'localization spin: {math.degrees(self.spin_angle):.0f} deg '
                               f'at {self.spin_speed} rad/s')
        t = Twist()
        t.angular.z = self.spin_speed
        end = time.monotonic() + duration

        def tick():
            if time.monotonic() < end and self.busy:
                self.pub_cmd.publish(t)
                return
            self.pub_cmd.publish(Twist())
            self.spin_timer.cancel()
            if self.busy:
                self.settle_timer = self.create_timer(1.0, self._after_settle)   # let AMCL settle
        self.spin_timer = self.create_timer(0.1, tick)

    def _after_settle(self):
        self.settle_timer.cancel()
        self.request_plan()

    def robot_pose(self):
        tf = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.orientation = tf.transform.rotation
        return ps

    def request_plan(self):
        if not self.plan_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('compute_path_to_pose not available - is Nav2 running?')
            self.busy = False
            return
        try:
            here = self.robot_pose()
        except Exception as e:
            self.get_logger().error(f'robot not localized ({self.map_frame} -> {self.base_frame}): {e}')
            self.busy = False
            return
        goal = ComputePathToPose.Goal()
        goal.goal = here          # SpiralSTC covers everything from the start; the goal is unused
        goal.start = here
        goal.use_start = True
        goal.planner_id = self.planner_id
        self.get_logger().info(f'planning full coverage from ({here.pose.position.x:.2f}, '
                               f'{here.pose.position.y:.2f}) with {self.planner_id}...')
        self.plan_client.send_goal_async(goal).add_done_callback(self.on_plan_accepted)

    def on_plan_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            self.get_logger().error('planner rejected the request')
            self.busy = False
            return
        h.get_result_async().add_done_callback(self.on_plan)

    def on_plan(self, fut):
        raw = fut.result().result.path
        path = densify(raw, self.step)
        if len(path.poses) < 2:
            self.get_logger().error('empty coverage plan: the robot is probably too close to a wall or obstacle '
                                    'for the planner. Move it into open space (0.5 m+ from walls) and call ~/start again.')
            self.busy = False
            return
        length = sum(math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y)
                     for a, b in zip(path.poses, path.poses[1:]))
        self.get_logger().info(f'coverage plan: {len(raw.poses)} corners -> {len(path.poses)} poses, {length:.1f} m')
        if length < 0.5:
            # Spiral-STC returns a stub when the robot's own start tile counts
            # as blocked (too close to a wall / obstacle in the global costmap).
            self.get_logger().error(
                'coverage plan is empty: the robot is probably too close to a wall or obstacle '
                'for the planner. Move it into open space (0.5 m+ from walls) and call '
                '~/start again.')
            self.busy = False
            return
        self.pub_plan.publish(path)
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('follow_path not available')
            self.busy = False
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        self.follow_client.send_goal_async(goal, feedback_callback=self.on_feedback) \
            .add_done_callback(self.on_follow_accepted)

    def on_follow_accepted(self, fut):
        self.follow_handle = fut.result()
        if not self.follow_handle.accepted:
            self.get_logger().error('controller rejected the coverage path')
            self.busy = False
            return
        self.follow_handle.get_result_async().add_done_callback(self.on_done)

    def on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'following: {fb.distance_to_goal:.1f} m to go, {fb.speed:.2f} m/s',
                               throttle_duration_sec=5.0)

    def on_done(self, fut):
        status = fut.result().status
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        self.get_logger().info(f'coverage finished: {names.get(status, status)}')
        self.pub_cmd.publish(Twist())
        self.busy = False
        self.follow_handle = None


def main():
    rclpy.init()
    node = CoverageRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub_cmd.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()
