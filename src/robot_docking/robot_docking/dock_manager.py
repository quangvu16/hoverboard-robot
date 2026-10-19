#!/usr/bin/env python3
"""Dock manager: the robot's one charging dock, docking, and when to go there.

The robot backs into its dock:

  dock    1. Nav2 (navigate_to_pose) drives forwards to the staging pose,
             staging_distance in front of the docked robot, facing away from
             the dock;
          2. a reverse controller here backs the rest of the way, steering on
             the TF pose, and stops within dock_tolerance of the dock.
  undock  drive forwards staging_distance, back to the staging pose.

Backwards because the front HC-SR04 reverses the robot away from anything
nearer than 350 mm (firmware sonar.h), and the collision monitor only watches
the front: both stay pointed away from the dock.

Not Nav2's docking server: its Humble release (opennav_docking 0.0.2) got
backwards docking wrong in simulation here - with one dock convention the robot
turned round at the staging pose, with the other it undocked by reversing into
the dock's wall. With a map-pose dock and no camera it would add nothing this
does not do. If a dock detector (AprilTag) is added later, revisit it.

Talks to the web page / app through topics, so rosbridge clients need no actions:

  /dock/command  std_msgs/String  in   "save"     the robot is in the dock now: remember this pose
                                       "dock" / "undock" / "cancel"
                                       "auto on" / "auto off"   dock by itself when the battery is low
  /dock/status   std_msgs/String  out  JSON, latched: state, message, dock {x, y, yaw} or null,
                                       auto, battery ("ok"/"low"/...), percentage
  /dock/pose     geometry_msgs/PoseStamped out, latched: the saved dock (map frame, the
                                       robot's pose when docked)
  /cmd_vel       geometry_msgs/Twist out, only during the reverse / undock moves: the same
                                       safety chain as Nav2 (collision monitor, twist_mux)

state is one of: no_dock, idle, docking, docked, undocking, failed.

Auto-dock: when the ESP32 reports "low" (it has already halved the speed,
firmware battery.h) and a dock is saved, the robot docks, preempting any Nav2
goal. Not at "critical": the ESP32 holds the wheels at zero then.

The dock pose is stored in dock_file (JSON) so it survives restarts.
"""
import json
import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DockManager(Node):
    def __init__(self):
        super().__init__('dock_manager')
        P = self.declare_parameter
        self.dock_file = os.path.expanduser(P('dock_file', '~/.ros/robot_dock.json').value)
        self.map_frame = P('map_frame', 'map').value
        self.base_frame = P('base_frame', 'base_footprint').value
        self.auto = bool(P('auto_dock', True).value)
        self.staging = float(P('staging_distance', 0.7).value)     # m in front of the docked robot
        self.v_max = float(P('approach_speed', 0.10).value)        # m/s, reversing in / driving out
        self.v_min = float(P('min_speed', 0.04).value)             # below this the wheels stall
        self.k_ang = float(P('k_angular', 1.5).value)
        self.w_max = float(P('max_angular', 0.6).value)            # rad/s
        self.tol = float(P('dock_tolerance', 0.01).value)          # m short of the dock along its axis
        self.move_timeout = float(P('move_timeout', 40.0).value)   # s, reverse in / drive out
        self.nav_timeout = float(P('nav_timeout', 180.0).value)    # s, to the staging pose
        self.retries = int(P('retries', 1).value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'dock/status', latched)
        self.pose_pub = self.create_publisher(PoseStamped, 'dock/pose', latched)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, 'dock/command', self.on_command, 10)
        self.create_subscription(String, 'hoverboard/battery_state', self.on_battery_state, latched)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.dock = self.load()           # robot pose when docked {x, y, yaw}, map frame
        self.state = 'idle' if self.dock else 'no_dock'
        self.message = '' if self.dock else 'No dock saved yet: park the robot in the dock and press "Set dock here".'
        self.phase = None                 # None, 'nav', 'reverse', 'forward'
        self.phase_t = None
        self.nav_goal = None
        self.seq = 0                      # results of replaced nav goals are ignored
        self.tries = 0
        self.battery = ''
        self.percentage = float('nan')
        self.timer = self.create_timer(0.05, self.control)   # 20 Hz while moving
        self.publish_dock()
        self.publish_status()
        self.get_logger().info(f'dock file {self.dock_file}: ' + (
            f'dock at ({self.dock["x"]:.2f}, {self.dock["y"]:.2f}, {math.degrees(self.dock["yaw"]):.0f} deg)'
            if self.dock else 'no dock saved') + f'; auto-dock {"on" if self.auto else "off"}')

    # ------------------------------------------------------------ persistence / publishing
    def load(self):
        try:
            with open(self.dock_file) as f:
                d = json.load(f)
            return {'x': float(d['x']), 'y': float(d['y']), 'yaw': float(d['yaw'])}
        except (OSError, ValueError, KeyError):
            return None

    def save(self):
        os.makedirs(os.path.dirname(self.dock_file) or '.', exist_ok=True)
        with open(self.dock_file, 'w') as f:
            json.dump({**self.dock, 'frame': self.map_frame}, f, indent=2)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'message': self.message, 'dock': self.dock, 'auto': self.auto,
            'battery': self.battery,
            'percentage': None if math.isnan(self.percentage) else round(self.percentage, 3)})))

    def publish_dock(self):
        if not self.dock:
            return
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = self.dock['x'], self.dock['y']
        p.pose.orientation.z, p.pose.orientation.w = math.sin(self.dock['yaw'] / 2), math.cos(self.dock['yaw'] / 2)
        self.pose_pub.publish(p)

    def set_state(self, state, message=''):
        changed = (state, message) != (self.state, self.message)
        self.state, self.message = state, message
        if changed and message:
            self.get_logger().info(f'{state}: {message}')
        self.publish_status()

    def robot_pose(self):
        t = self.tf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def staging_pose(self):
        d = self.dock
        return (d['x'] + self.staging * math.cos(d['yaw']), d['y'] + self.staging * math.sin(d['yaw']), d['yaw'])

    # ------------------------------------------------------------ commands
    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'save':
            self.save_here()
        elif cmd == 'dock':
            self.start_dock('requested')
        elif cmd == 'undock':
            self.start_undock()
        elif cmd == 'cancel':
            self.stop('idle', 'Cancelled.')
        elif cmd in ('auto on', 'auto off'):
            self.auto = cmd == 'auto on'
            self.set_state(self.state, f'Auto-dock {"on" if self.auto else "off"}.')
        else:
            self.get_logger().warn(f'unknown /dock/command "{msg.data}"')

    def save_here(self):
        if self.phase:
            self.stop(self.state, '')
        try:
            x, y, yaw = self.robot_pose()
        except TransformException as e:
            self.set_state(self.state, f'Cannot save the dock: no {self.map_frame} -> {self.base_frame} ({e}).')
            return
        self.dock = {'x': x, 'y': y, 'yaw': yaw}
        try:
            self.save()
        except OSError as e:
            self.get_logger().error(f'cannot write {self.dock_file}: {e}')
        self.publish_dock()
        self.set_state('docked', f'Dock saved at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg).')

    def start_dock(self, why):
        if not self.dock:
            self.set_state('no_dock', 'No dock saved yet: park the robot in the dock and press "Set dock here".')
            return
        if self.state == 'docking':
            return
        self.tries = 0
        self.set_state('docking', f'Docking ({why}): driving to the staging pose.')
        self.go_to_staging()

    def go_to_staging(self):
        if not self.nav.wait_for_server(timeout_sec=1.0):
            self.stop('failed', 'Nav2 is not running (navigate_to_pose): start a navigation stack.')
            return
        sx, sy, syaw = self.staging_pose()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = sx, sy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(syaw / 2), math.cos(syaw / 2)
        self.seq += 1
        seq = self.seq
        self.phase, self.phase_t = 'nav', self.get_clock().now()
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_nav_accepted(f, seq))

    def on_nav_accepted(self, future, seq):
        handle = future.result()
        if seq != self.seq or self.phase != 'nav':
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.stop('failed', 'Nav2 rejected the staging pose.')
            return
        self.nav_goal = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_nav_result(f, seq))

    def on_nav_result(self, future, seq):
        if seq != self.seq or self.phase != 'nav':
            return
        self.nav_goal = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.phase, self.phase_t = 'reverse', self.get_clock().now()
            self.set_state('docking', 'Reversing into the dock.')
        else:
            self.stop('failed', 'Could not reach the staging pose in front of the dock.')

    def start_undock(self):
        if not self.dock:
            self.set_state('no_dock', 'No dock saved yet.')
            return
        if self.state in ('docking', 'undocking'):
            return
        self.phase, self.phase_t = 'forward', self.get_clock().now()
        self.set_state('undocking', 'Driving out of the dock.')

    def stop(self, state, message):
        self.seq += 1                      # orphan any nav callbacks
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None
        if self.phase in ('reverse', 'forward'):
            for _ in range(3):
                self.cmd_pub.publish(Twist())
        self.phase = None
        self.set_state(state, message)

    # ------------------------------------------------------------ the two short moves
    def control(self):
        if self.phase not in ('reverse', 'forward'):
            if self.phase == 'nav' and (self.get_clock().now() - self.phase_t).nanoseconds * 1e-9 > self.nav_timeout:
                self.stop('failed', 'Timed out driving to the staging pose.')
            return
        if (self.get_clock().now() - self.phase_t).nanoseconds * 1e-9 > self.move_timeout:
            self.retry_or_fail('timed out')
            return
        try:
            x, y, yaw = self.robot_pose()
        except TransformException:
            return
        d = self.dock
        ux, uy = math.cos(d['yaw']), math.sin(d['yaw'])        # dock axis, pointing out of the dock
        s = (x - d['x']) * ux + (y - d['y']) * uy              # distance out of the dock along the axis
        cmd = Twist()
        if self.phase == 'reverse':
            if s <= self.tol:
                self.stop('docked', f'Docked ({abs(s) * 100:.0f} cm along, {self.lateral(x, y) * 100:.0f} cm '
                                    f'off the axis, {math.degrees(wrap(yaw - d["yaw"])):.0f} deg).')
                return
            # Aim the rear at a point on the axis a little behind the robot's
            # projection, which steers it back onto the axis while reversing.
            look = max(0.15, min(0.4, s))
            tx, ty = d['x'] + (s - look) * ux, d['y'] + (s - look) * uy
            a = wrap(math.atan2(ty - y, tx - x) - (yaw + math.pi))   # 0 = target straight behind
            if abs(a) > 0.6:                                          # badly off: turn first
                cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_ang * a))
            else:
                cmd.linear.x = -max(self.v_min, min(self.v_max, 0.5 * s))
                cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_ang * a))
        else:   # forward, out to the staging pose
            if s >= self.staging - self.tol:
                self.stop('idle', 'Undocked.')
                return
            tx, ty = d['x'] + min(self.staging, s + 0.3) * ux, d['y'] + min(self.staging, s + 0.3) * uy
            a = wrap(math.atan2(ty - y, tx - x) - yaw)
            cmd.linear.x = max(self.v_min, min(self.v_max, 0.5 * (self.staging - s)))
            cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_ang * a))
        self.cmd_pub.publish(cmd)

    def lateral(self, x, y):
        d = self.dock
        return abs(-(x - d['x']) * math.sin(d['yaw']) + (y - d['y']) * math.cos(d['yaw']))

    def retry_or_fail(self, why):
        if self.phase == 'reverse' and self.tries < self.retries:
            self.tries += 1
            for _ in range(3):
                self.cmd_pub.publish(Twist())
            self.set_state('docking', f'Docking {why}; back to the staging pose, try {self.tries + 1}.')
            self.go_to_staging()
        else:
            self.stop('failed', f'{"Docking" if self.phase == "reverse" else "Undocking"} {why}.')

    # ------------------------------------------------------------ battery
    def on_battery_state(self, msg):
        prev, self.battery = self.battery, msg.data
        if self.battery == 'low' and prev != 'low' and self.auto:
            if self.state in ('docking', 'docked'):
                self.publish_status()
            elif self.dock is None:
                self.set_state(self.state, 'Battery low, but no dock saved: charge it by hand.')
            else:
                self.start_dock('battery low')
        else:
            self.publish_status()

    def on_battery(self, msg):
        p = msg.percentage
        old = self.percentage
        if math.isnan(p) != math.isnan(old) or (not math.isnan(p) and abs(p - old) >= 0.01):
            self.percentage = p
            self.publish_status()


def main():
    rclpy.init()
    node = DockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
