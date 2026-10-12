#!/usr/bin/env python3
"""Plain topics for the web page, so the browser never has to speak actions or TF.

The page talks to rosbridge (websocket, port 9090) with roslib.js; rosbridge
turns JSON into ROS messages. Nav2's NavigateToPose action and the TF tree are
awkward to use from a browser, so this node flattens them:

  /web/goal        geometry_msgs/PoseStamped  in   send a NavigateToPose goal
                                                   (a new goal replaces the old one)
  /web/cancel      std_msgs/Empty             in   cancel the active goal
  /web/robot_pose  geometry_msgs/PoseStamped  out  global_frame -> base_frame, pose_rate_hz
  /web/nav_status  std_msgs/String (JSON)     out  on every change and at 2 Hz:
      {"state": "idle|sending|active|succeeded|aborted|canceled|rejected|unavailable",
       "stack": "...", "goal": {"x","y","yaw"} | null, "distance_remaining",
       "eta_s", "nav_time_s", "recoveries", "message", "server_ready"}

The initial pose needs no help: the page publishes /initialpose directly
(AMCL reads it; Cartographer gets it through robot_cartographer's
initialpose_relay when a .pbstream map is loaded).

Parameters: global_frame (map), base_frame (base_footprint), pose_rate_hz (5),
action_name (navigate_to_pose), stack (shown on the page only).
"""
import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

FINAL_STATES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WebNavBridge(Node):

    def __init__(self):
        super().__init__('web_nav_bridge')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = float(self.declare_parameter('pose_rate_hz', 5.0).value)
        action_name = self.declare_parameter('action_name', 'navigate_to_pose').value
        self.stack = self.declare_parameter('stack', 'unknown').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, action_name)

        self.pose_pub = self.create_publisher(PoseStamped, 'web/robot_pose', 10)
        self.status_pub = self.create_publisher(String, 'web/nav_status', 10)
        self.create_subscription(PoseStamped, 'web/goal', self.on_goal, 10)
        self.create_subscription(Empty, 'web/cancel', self.on_cancel, 10)

        self.goal_handle = None
        self.goal_seq = 0          # ignores callbacks from a goal that was replaced
        self.status = {'state': 'idle', 'goal': None, 'message': ''}
        self.feedback = {}
        self.tf_warned = False

        self.create_timer(1.0 / rate, self.publish_pose)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'web_nav_bridge up: {self.global_frame} -> {self.base_frame}, '
            f'action {action_name}, stack {self.stack}')

    # ---------------------------------------------------------------- pose
    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException as e:
            if not self.tf_warned:
                self.get_logger().warn(f'no {self.global_frame} -> {self.base_frame} yet: {e}')
                self.tf_warned = True
            return
        self.tf_warned = False
        msg = PoseStamped()
        msg.header = t.header
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pose_pub.publish(msg)

    # ---------------------------------------------------------------- status
    def set_state(self, state, message=''):
        self.status['state'] = state
        self.status['message'] = message
        if state != 'active':
            self.feedback = {}
        self.publish_status()

    def publish_status(self):
        if self.status['state'] == 'unavailable' and self.nav.server_is_ready():
            self.status.update(state='idle', message='Nav2 is up')
        out = dict(self.status)
        out.update(self.feedback)
        out['stack'] = self.stack
        out['server_ready'] = self.nav.server_is_ready()
        self.status_pub.publish(String(data=json.dumps(out)))

    # ---------------------------------------------------------------- goals
    def on_goal(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = self.global_frame
        p, q = msg.pose.position, msg.pose.orientation
        self.status['goal'] = {'x': round(p.x, 3), 'y': round(p.y, 3),
                               'yaw': round(yaw_of(q), 3)}
        if not self.nav.server_is_ready():
            self.set_state('unavailable', 'Nav2 is not running (no navigate_to_pose server)')
            return
        self.goal_seq += 1
        seq = self.goal_seq
        self.get_logger().info(
            f'goal #{seq}: x={p.x:.2f} y={p.y:.2f} yaw={math.degrees(yaw_of(q)):.0f} deg')
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.set_state('sending')
        future = self.nav.send_goal_async(
            goal, feedback_callback=lambda fb, s=seq: self.on_feedback(fb, s))
        future.add_done_callback(lambda f, s=seq: self.on_accepted(f, s))

    def on_accepted(self, future, seq):
        if seq != self.goal_seq:
            return
        handle = future.result()
        if not handle.accepted:
            self.set_state('rejected', 'Nav2 rejected the goal')
            return
        self.goal_handle = handle
        self.set_state('active')
        handle.get_result_async().add_done_callback(lambda f, s=seq: self.on_result(f, s))

    def on_feedback(self, fb, seq):
        if seq != self.goal_seq or self.status['state'] not in ('sending', 'active'):
            return
        f = fb.feedback
        eta = f.estimated_time_remaining.sec + f.estimated_time_remaining.nanosec * 1e-9
        nav = f.navigation_time.sec + f.navigation_time.nanosec * 1e-9
        self.feedback = {'distance_remaining': round(f.distance_remaining, 2),
                         'eta_s': round(eta, 1), 'nav_time_s': round(nav, 1),
                         'recoveries': f.number_of_recoveries}

    def on_result(self, future, seq):
        if seq != self.goal_seq:
            return             # a newer goal replaced this one; its result is noise
        status = future.result().status
        state = FINAL_STATES.get(status, 'aborted')
        self.goal_handle = None
        self.get_logger().info(f'goal #{seq}: {state}')
        self.set_state(state)

    def on_cancel(self, _msg):
        if self.goal_handle is None:
            self.set_state('idle', 'nothing to cancel')
            return
        self.get_logger().info(f'cancel goal #{self.goal_seq}')
        self.goal_handle.cancel_goal_async()


def main():
    rclpy.init()
    node = WebNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
