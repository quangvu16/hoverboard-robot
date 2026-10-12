"""Formation slots behind/beside the leader (Yahboom's "queue", for hoverboards).

Broadcasts TF frames point1 and point2 relative to the leader's base frame,
continuously at 10 Hz (Yahboom sent them once per /team message, so they went
stale at once). Followers (formation_follower) drive to them.

    ros2 run hoverboard_multi formation_leader
    ros2 topic pub --once /team std_msgs/msg/String "data: horizontal"

Formations (/team or the Team parameter):
    vertical    in a line behind the leader      point1 (-d, 0), point2 (-2d, 0)
    horizontal  side by side                     point1 (0, +d), point2 (0, -d)
    convoy      V behind the leader              point1 (-d, +d), point2 (-d, -d)
Parameters: leader_frame robot1/base_footprint, Team vertical, Distance 0.8 m
(a hoverboard base is 0.33 m square; Yahboom's 0.5 m leaves ~0.17 m between).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

SLOTS = {
    'vertical': lambda d: ((-d, 0.0), (-2 * d, 0.0)),
    'horizontal': lambda d: ((0.0, d), (0.0, -d)),
    'convoy': lambda d: ((-d, d), (-d, -d)),
}
SLOTS['convoyl'] = SLOTS['convoy']   # Yahboom's spelling


class FormationLeader(Node):
    def __init__(self):
        super().__init__('formation_leader')
        self.leader = self.declare_parameter('leader_frame', 'robot1/base_footprint').value
        self.declare_parameter('Team', 'vertical')
        self.declare_parameter('Distance', 0.8)
        self.br = TransformBroadcaster(self)
        self.create_subscription(String, '/team', self.on_team, 1)
        self.create_timer(0.1, self.broadcast)

    def on_team(self, msg):
        if msg.data in SLOTS:
            self.set_parameters([rclpy.parameter.Parameter('Team', value=msg.data)])
            self.get_logger().info(f'formation: {msg.data}')
        else:
            self.get_logger().warn(f'unknown formation "{msg.data}"; use one of {sorted(SLOTS)}')

    def broadcast(self):
        team = self.get_parameter('Team').value
        d = float(self.get_parameter('Distance').value)
        slots = SLOTS.get(team, SLOTS['vertical'])(d)
        now = self.get_clock().now().to_msg()
        for name, (x, y) in zip(('point1', 'point2'), slots):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.leader
            t.child_frame_id = name
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FormationLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
