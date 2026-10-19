#!/usr/bin/env python3
"""Plain topics for the web page, so the browser never has to speak actions or TF.

The page talks to rosbridge (websocket, port 9090) with roslib.js; rosbridge
turns JSON into ROS messages. Nav2's NavigateToPose action and the TF tree are
awkward to use from a browser, so this node flattens them:

  /web/goal        geometry_msgs/PoseStamped  in   send a NavigateToPose goal
                                                   (a new goal replaces the old one)
  /web/cancel      std_msgs/Empty             in   cancel the active goal
  /web/robot_pose  geometry_msgs/PoseStamped  out  global_frame -> base_frame, pose_rate_hz
  /web/explore     std_msgs/String            in   "start" / "stop": explore_lite on
                                                   /explore/resume (it runs paused while mapping)
  /web/patrol      std_msgs/String (JSON)     in   {"spots": [{"name","x","y","yaw"}, ...],
                                                    "loops": n (0 = until stopped), "wait_s": s}
                                                   or "stop"; visits the spots in order, one
                                                   NavigateToPose goal each, from this node, so it
                                                   keeps going with the page closed
  /web/nav_status  std_msgs/String (JSON)     out  on every change and at 2 Hz:
      {"state": "idle|sending|active|succeeded|aborted|canceled|rejected|unavailable",
       "stack": "...", "goal": {"x","y","yaw"} | null, "distance_remaining",
       "eta_s", "nav_time_s", "recoveries", "message", "server_ready",
       "explore": "unavailable|paused|exploring|finished",
       "patrol": null | {"index", "count", "name", "loop", "loops", "waiting_s"}}

Who wins Nav2: the last to send a goal. So a goal from the page, Cancel, or a
patrol pauses exploring; a goal or Cancel stops the patrol; and docking
(robot_docking, /dock/status) stops the patrol and is never retried over.

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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String
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
        self.create_subscription(String, 'web/explore', self.on_explore, 10)
        self.create_subscription(String, 'web/patrol', self.on_patrol, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.explore_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.create_subscription(Bool, 'explore/finished', self.on_explore_finished, latched)
        self.create_subscription(String, 'dock/status', self.on_dock_status, latched)
        self.explore = 'paused'      # what we last told explore_lite, or 'finished'
        self.patrol = None           # {spots, loops, wait_s, index, loop, retried}
        self.patrol_timer = None
        self.patrol_wait_until = None
        self.dock_state = ''

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
        out['explore'] = self.explore if self.count_subscribers('explore/resume') > 0 else 'unavailable'
        out['patrol'] = self.patrol_status()
        if self.patrol is not None and not out.get('message'):
            out['message'] = self.patrol_text()
        self.status_pub.publish(String(data=json.dumps(out)))

    # ---------------------------------------------------------------- goals
    def on_goal(self, msg):
        # A goal from the page overrides exploring and any patrol.
        self.pause_explore()
        self.stop_patrol('')
        self.send_nav(msg)

    def send_nav(self, msg):
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
        if state == 'aborted' and self.dock_state in ('docking', 'undocking'):
            self.set_state('canceled', 'Replaced: the robot is docking')   # not a failure
        else:
            self.set_state(state)
        if self.patrol is not None:
            self.patrol_result(state)

    def on_cancel(self, _msg):
        self.pause_explore()
        self.stop_patrol('Patrol stopped.')
        if self.goal_handle is None:
            self.set_state('idle', 'nothing to cancel')
            return
        self.get_logger().info(f'cancel goal #{self.goal_seq}')
        self.goal_handle.cancel_goal_async()

    # ---------------------------------------------------------------- explore
    def on_explore(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            if self.count_subscribers('explore/resume') == 0:
                self.set_state(self.status['state'], 'Explore is not running: launch web_nav with '
                               'stack:=cartographer and no map:= (mapping mode)')
                return
            self.stop_patrol('')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.explore = 'exploring'
            self.explore_pub.publish(Bool(data=True))
            self.set_state(self.status['state'], 'Exploring: the robot maps the unknown areas by itself')
        elif cmd == 'stop':
            self.pause_explore()
            self.set_state(self.status['state'], 'Exploring stopped')

    def pause_explore(self):
        if self.explore == 'exploring':
            self.explore = 'paused'
            self.explore_pub.publish(Bool(data=False))

    def on_explore_finished(self, msg):
        if msg.data and self.explore == 'exploring':
            self.explore = 'finished'
            self.set_state(self.status['state'], 'Exploring finished: no unknown areas left. Save the map.')

    # ---------------------------------------------------------------- patrol
    def on_patrol(self, msg):
        if msg.data.strip().lower() == 'stop':
            had = self.patrol is not None
            self.stop_patrol('Patrol stopped.')
            if had and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            return
        try:
            req = json.loads(msg.data)
            spots = [{'name': str(s.get('name', f'spot {i + 1}')), 'x': float(s['x']),
                      'y': float(s['y']), 'yaw': float(s.get('yaw', 0.0))}
                     for i, s in enumerate(req['spots'])]
        except (ValueError, KeyError, TypeError) as e:
            self.set_state(self.status['state'], f'Bad patrol request: {e}')
            return
        if len(spots) < 1:
            self.set_state(self.status['state'], 'A patrol needs at least one saved spot')
            return
        self.pause_explore()
        self.stop_patrol('')
        self.patrol = {'spots': spots, 'loops': max(0, int(req.get('loops', 1))),
                       'wait_s': max(0.0, float(req.get('wait_s', 5.0))),
                       'index': 0, 'loop': 1, 'retried': False}
        self.get_logger().info(f'patrol: {len(spots)} spots, loops {self.patrol["loops"] or "forever"}, '
                               f'wait {self.patrol["wait_s"]:.0f} s')
        self.patrol_go()

    def patrol_status(self):
        p = self.patrol
        if p is None:
            return None
        waiting = None
        if self.patrol_wait_until is not None:
            waiting = max(0.0, round((self.patrol_wait_until - self.get_clock().now()).nanoseconds * 1e-9, 1))
        return {'index': p['index'], 'count': len(p['spots']), 'name': p['spots'][p['index']]['name'],
                'loop': p['loop'], 'loops': p['loops'], 'waiting_s': waiting}

    def patrol_go(self):
        p = self.patrol
        s = p['spots'][p['index']]
        self.patrol_wait_until = None
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame
        msg.pose.position.x, msg.pose.position.y = s['x'], s['y']
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(s['yaw'] / 2), math.cos(s['yaw'] / 2)
        self.send_nav(msg)

    def patrol_text(self):
        p = self.patrol
        s = p['spots'][p['index']]
        where = (f'{s["name"]} ({p["index"] + 1}/{len(p["spots"])}, '
                 f'loop {p["loop"]}{"/" + str(p["loops"]) if p["loops"] else ""})')
        if self.patrol_wait_until is not None:
            left = max(0.0, (self.patrol_wait_until - self.get_clock().now()).nanoseconds * 1e-9)
            return f'Patrol: waiting {left:.0f} s, then {where}'
        return f'Patrol: to {where}'

    def patrol_result(self, state):
        p = self.patrol
        if self.dock_state in ('docking', 'undocking'):
            self.stop_patrol('Patrol stopped: the robot is docking.')
            return
        if state == 'canceled':
            self.stop_patrol('Patrol stopped.')
            return
        if state != 'succeeded':
            if not p['retried']:              # one more try at the same spot
                p['retried'] = True
                self.start_patrol_timer(2.0)
                return
            self.get_logger().warn(f'patrol: {p["spots"][p["index"]]["name"]} unreachable, skipping')
        p['retried'] = False
        p['index'] += 1
        if p['index'] >= len(p['spots']):
            p['index'] = 0
            p['loop'] += 1
            if p['loops'] and p['loop'] > p['loops']:
                self.stop_patrol(f'Patrol done: {p["loops"]} loop{"s" if p["loops"] > 1 else ""}.')
                return
        self.start_patrol_timer(p['wait_s'])

    def start_patrol_timer(self, wait_s):
        if self.patrol_timer is not None:
            self.patrol_timer.cancel()
        self.patrol_wait_until = self.get_clock().now() + Duration(seconds=wait_s)
        self.patrol_timer = self.create_timer(max(0.05, wait_s), self.on_patrol_timer)
        self.publish_status()

    def on_patrol_timer(self):
        self.patrol_timer.cancel()
        self.patrol_timer = None
        if self.patrol is not None:
            self.patrol_go()

    def stop_patrol(self, message):
        if self.patrol_timer is not None:
            self.patrol_timer.cancel()
            self.patrol_timer = None
        self.patrol_wait_until = None
        if self.patrol is not None:
            self.patrol = None
            if message:
                self.get_logger().info(message)
                self.set_state(self.status['state'], message)

    # ---------------------------------------------------------------- dock
    def on_dock_status(self, msg):
        try:
            state = json.loads(msg.data).get('state', '')
        except ValueError:
            return
        if state in ('docking', 'undocking') and self.dock_state not in ('docking', 'undocking'):
            self.pause_explore()
            if self.patrol is not None:
                self.stop_patrol('Patrol stopped: the robot is docking.')
        self.dock_state = state


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
