#!/usr/bin/env python3
"""RViz "2D Pose Estimate" for Cartographer pure localization.

Cartographer ignores /initialpose. This node turns it into what Cartographer
understands: finish the active trajectory, then start a new one at the clicked
pose, relative to the frozen map (trajectory 0). Started by
cartographer_nav.launch.py only when a map:=...pbstream is loaded.

Parameters:
  configuration_directory, configuration_basename   the localization .lua
  relative_to_trajectory_id                         the frozen map, 0
"""
import rclpy
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPoseRelay(Node):

    def __init__(self):
        super().__init__('initialpose_relay')
        self.config_dir = self.declare_parameter('configuration_directory', '').value
        self.config_base = self.declare_parameter(
            'configuration_basename', 'robot_2d_localization.lua').value
        self.map_traj = self.declare_parameter('relative_to_trajectory_id', 0).value

        self.get_states = self.create_client(GetTrajectoryStates, 'get_trajectory_states')
        self.finish = self.create_client(FinishTrajectory, 'finish_trajectory')
        self.start = self.create_client(StartTrajectory, 'start_trajectory')
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_pose, 10)
        self.busy = False

    def on_pose(self, msg):
        if self.busy:
            self.get_logger().warn('previous 2D Pose Estimate still being applied, ignored')
            return
        if msg.header.frame_id not in ('map', '/map'):
            self.get_logger().warn(f'initialpose in frame "{msg.header.frame_id}", expected map')
            return
        if not self.get_states.service_is_ready():
            self.get_logger().error('cartographer_node is not running')
            return
        self.busy = True
        self.pose = msg.pose.pose
        self.get_states.call_async(GetTrajectoryStates.Request()).add_done_callback(self.on_states)

    def on_states(self, future):
        st = future.result().trajectory_states
        self.pending = [tid for tid, s in zip(st.trajectory_id, st.trajectory_state)
                        if s == TrajectoryStates.ACTIVE]
        self.finish_next()

    def finish_next(self):
        if not self.pending:
            self.start_new()
            return
        tid = self.pending.pop()
        req = FinishTrajectory.Request(trajectory_id=tid)
        self.finish.call_async(req).add_done_callback(lambda f: self.finish_next())

    def start_new(self):
        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_base
        req.use_initial_pose = True
        req.initial_pose = self.pose
        req.relative_to_trajectory_id = self.map_traj
        self.start.call_async(req).add_done_callback(self.on_started)

    def on_started(self, future):
        self.busy = False
        res = future.result()
        p = self.pose.position
        if res.status.code == 0:
            self.get_logger().info(
                f'localizing from x={p.x:.2f} y={p.y:.2f} (trajectory {res.trajectory_id})')
        else:
            self.get_logger().error(f'start_trajectory failed: {res.status.message}')


def main():
    rclpy.init()
    rclpy.spin(InitialPoseRelay())


if __name__ == '__main__':
    main()
