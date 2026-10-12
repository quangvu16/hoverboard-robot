#!/usr/bin/env python3
"""Bridge between the ESP32 hoverboard firmware and ROS 2.

Receives "HBT1" telemetry datagrams from the ESP32, turns the accumulated wheel
ticks into odometry, and sends "HBC1" wheel-RPM commands back in response to
/cmd_vel.

The wire format is defined in the firmware, ldrobot_ld14p_esp32_ros.ino, in the
"hoverboard link" section. Keep the two in step.

Why RPM on the wire and not m/s: the ESP32 knows nothing about the robot. All
geometry (wheel diameter, track, encoder ticks per revolution) lives here as
ROS parameters, so it can be measured and corrected without reflashing.

Why accumulated ticks and not deltas: UDP drops packets. A running total means
a dropped datagram costs nothing - the next one still carries the truth. A
stream of deltas would lose distance permanently on every drop.
"""

import math
import socket
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, BatteryState
from tf2_ros import TransformBroadcaster

# --- wire format, must match the firmware -----------------------------------
TELEM_MAGIC = b'HBT1'
TELEM_FMT = '<4sHIHHiihhhhhhII'
TELEM_LEN = struct.calcsize(TELEM_FMT)

CMD_MAGIC = b'HBC1'
CMD_FMT = '<4sHhhH'
CMD_LEN = struct.calcsize(CMD_FMT)

FLAG_LINK_UP = 0x0001
FLAG_CMD_TIMEOUT = 0x0002
FLAG_HAS_CURRENT = 0x0004

CMD_FLAG_ENABLE = 0x0001

assert TELEM_LEN == 42, TELEM_LEN
assert CMD_LEN == 12, CMD_LEN


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class HoverboardUdpBridge(Node):

    def __init__(self):
        super().__init__('hoverboard_udp_bridge')

        self.declare_parameter('esp32_ip', '192.168.1.6')
        self.declare_parameter('cmd_port', 9901)      # we send here
        self.declare_parameter('telem_port', 9902)    # we listen here
        self.declare_parameter('wheel_diameter', 0.165)
        self.declare_parameter('wheel_track', 0.5)
        self.declare_parameter('encoder_tpr', 90)
        self.declare_parameter('max_wheel_rpm', 150.0)
        self.declare_parameter('cmd_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        g = self.get_parameter
        self.esp32_ip = g('esp32_ip').value
        self.cmd_port = int(g('cmd_port').value)
        self.telem_port = int(g('telem_port').value)
        self.wheel_radius = float(g('wheel_diameter').value) / 2.0
        self.track = float(g('wheel_track').value)
        self.tpr = int(g('encoder_tpr').value)
        self.max_rpm = float(g('max_wheel_rpm').value)
        self.cmd_vel_timeout = float(g('cmd_vel_timeout').value)
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.publish_tf = bool(g('publish_tf').value)
        self.left_joint = g('left_wheel_joint').value
        self.right_joint = g('right_wheel_joint').value
        self.sign_left = -1.0 if g('invert_left').value else 1.0
        self.sign_right = -1.0 if g('invert_right').value else 1.0

        # Metres of ground travel per encoder tick.
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.tpr

        # --- odometry state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks_l = None
        self.last_ticks_r = None
        self.last_board_ms = None
        self.last_telem_stamp = None
        self.telem_count = 0
        self.telem_bad = 0
        self.warned_no_telem = False

        # --- command state ---
        self.target_rpm_l = 0.0
        self.target_rpm_r = 0.0
        self.last_cmd_vel_time = None
        self.cmd_seq = 0

        # --- sockets ---
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(('0.0.0.0', self.telem_port))
        self.rx_sock.setblocking(False)
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', sensor_qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', sensor_qos)
        self.batt_pub = self.create_publisher(BatteryState, 'battery_state', sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        # Drain the socket well above the 50 Hz telemetry rate so datagrams do
        # not queue up and age.
        self.create_timer(0.005, self.poll_telemetry)
        self.create_timer(1.0 / float(g('cmd_rate').value), self.send_command)

        self.get_logger().info(
            f'listening for telemetry on 0.0.0.0:{self.telem_port}, '
            f'commands to {self.esp32_ip}:{self.cmd_port}')
        self.get_logger().info(
            f'wheel radius {self.wheel_radius:.4f} m, track {self.track:.3f} m, '
            f'{self.tpr} ticks/rev -> {self.m_per_tick * 1000:.3f} mm/tick')

    # ------------------------------------------------------------- commands
    def on_cmd_vel(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive: each wheel carries the body velocity plus half
        # the rotation, in opposite senses.
        v_left = v - w * self.track / 2.0
        v_right = v + w * self.track / 2.0

        rpm_l = v_left / (2.0 * math.pi * self.wheel_radius) * 60.0
        rpm_r = v_right / (2.0 * math.pi * self.wheel_radius) * 60.0

        # Clamp while preserving the ratio, so a too-fast turn still turns at
        # the right curvature instead of straightening out.
        peak = max(abs(rpm_l), abs(rpm_r))
        if peak > self.max_rpm and peak > 0.0:
            scale = self.max_rpm / peak
            rpm_l *= scale
            rpm_r *= scale

        self.target_rpm_l = rpm_l
        self.target_rpm_r = rpm_r
        self.last_cmd_vel_time = self.get_clock().now()

    def send_command(self):
        """Send the setpoint at a steady rate, whatever /cmd_vel is doing.

        The firmware stops the wheels if commands stop arriving, and the
        hoverboard mainboard faults if its own link goes quiet, so silence is
        never the right way to say "keep doing what you are doing".
        """
        enable = True
        if self.last_cmd_vel_time is None:
            enable = False
        else:
            age = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds * 1e-9
            if age > self.cmd_vel_timeout:
                enable = False

        if enable:
            rpm_l = self.target_rpm_l * self.sign_left
            rpm_r = self.target_rpm_r * self.sign_right
        else:
            rpm_l = 0.0
            rpm_r = 0.0

        pkt = struct.pack(
            CMD_FMT, CMD_MAGIC, self.cmd_seq & 0xFFFF,
            int(round(max(-32768, min(32767, rpm_l)))),
            int(round(max(-32768, min(32767, rpm_r)))),
            CMD_FLAG_ENABLE if enable else 0)
        self.cmd_seq += 1

        try:
            self.tx_sock.sendto(pkt, (self.esp32_ip, self.cmd_port))
        except OSError as exc:
            self.get_logger().warn(f'command send failed: {exc}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------ telemetry
    def poll_telemetry(self):
        newest = None
        while True:
            try:
                data, _addr = self.rx_sock.recvfrom(256)
            except BlockingIOError:
                break
            except OSError:
                break
            if len(data) < TELEM_LEN or not data.startswith(TELEM_MAGIC):
                self.telem_bad += 1
                continue
            newest = data[:TELEM_LEN]

        if newest is None:
            return
        self.handle_telemetry(struct.unpack(TELEM_FMT, newest))

    def handle_telemetry(self, t):
        (_magic, _seq, board_ms, flags, fb_bytes,
         ticks_l, ticks_r, rpm_l, rpm_r,
         batt_cv, temp_dc, curr_l_ca, curr_r_ca,
         _frames, _cksum_err) = t

        self.telem_count += 1
        now = self.get_clock().now()

        ticks_l = int(ticks_l * self.sign_left)
        ticks_r = int(ticks_r * self.sign_right)

        # The ESP32 rebooting resets its accumulator to zero. Its millis()
        # going backwards is the giveaway; re-baseline rather than integrating
        # the whole distance back to the origin in one step.
        rebooted = (self.last_board_ms is not None and board_ms < self.last_board_ms)
        self.last_board_ms = board_ms

        if self.last_ticks_l is None or rebooted:
            if rebooted:
                self.get_logger().warn('ESP32 restarted, re-baselining odometry')
            self.last_ticks_l = ticks_l
            self.last_ticks_r = ticks_r
            self.last_telem_stamp = now
            return

        d_left = (ticks_l - self.last_ticks_l) * self.m_per_tick
        d_right = (ticks_r - self.last_ticks_r) * self.m_per_tick
        self.last_ticks_l = ticks_l
        self.last_ticks_r = ticks_r

        dt = (now - self.last_telem_stamp).nanoseconds * 1e-9
        self.last_telem_stamp = now
        if dt <= 0.0:
            return

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.track

        # Integrate at the midpoint heading: over one 20 ms step the arc and
        # the chord differ negligibly, but the midpoint keeps a long constant
        # turn from bowing outward.
        mid = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(mid)
        self.y += d_center * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

        # Body velocities from the mainboard's own signed RPM, which is less
        # quantised than differentiating 90-tick-per-rev counters.
        v_left = rpm_l * self.sign_left / 60.0 * 2.0 * math.pi * self.wheel_radius
        v_right = rpm_r * self.sign_right / 60.0 * 2.0 * math.pi * self.wheel_radius
        vx = (v_left + v_right) / 2.0
        wz = (v_right - v_left) / self.track

        stamp = now.to_msg()
        self.publish_odom(stamp, vx, wz)
        self.publish_joints(stamp, ticks_l, ticks_r, rpm_l, rpm_r)
        self.publish_battery(stamp, batt_cv, curr_l_ca, curr_r_ca)

        if flags & FLAG_CMD_TIMEOUT:
            self.get_logger().warn(
                'firmware reports command timeout - wheels stopped',
                throttle_duration_sec=5.0)
        if not (flags & FLAG_LINK_UP):
            self.get_logger().warn(
                f'ESP32 reports no hoverboard feedback (frame size {fb_bytes})',
                throttle_duration_sec=5.0)

    def publish_odom(self, stamp, vx, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        # Wheel odometry is decent in x and yaw and meaningless sideways.
        # These are the conventional placeholders; tune if you run an EKF.
        odom.pose.covariance[0] = 0.001     # x
        odom.pose.covariance[7] = 0.001     # y
        odom.pose.covariance[35] = 0.01     # yaw
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.01
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(tf)

    def publish_joints(self, stamp, ticks_l, ticks_r, rpm_l, rpm_r):
        js = JointState()
        js.header.stamp = stamp
        js.name = [self.left_joint, self.right_joint]
        js.position = [ticks_l * 2.0 * math.pi / self.tpr,
                       ticks_r * 2.0 * math.pi / self.tpr]
        js.velocity = [rpm_l * self.sign_left * 2.0 * math.pi / 60.0,
                       rpm_r * self.sign_right * 2.0 * math.pi / 60.0]
        self.joint_pub.publish(js)

    def publish_battery(self, stamp, batt_cv, curr_l_ca, curr_r_ca):
        bs = BatteryState()
        bs.header.stamp = stamp
        bs.voltage = batt_cv / 100.0
        bs.current = -(curr_l_ca + curr_r_ca) / 100.0   # ROS sign: - = draining
        bs.percentage = float('nan')
        bs.present = batt_cv > 0
        bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        bs.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.batt_pub.publish(bs)

    def destroy_node(self):
        # Leave the robot stopped, not coasting on the last setpoint.
        try:
            stop = struct.pack(CMD_FMT, CMD_MAGIC, 0xFFFF, 0, 0, 0)
            for _ in range(3):
                self.tx_sock.sendto(stop, (self.esp32_ip, self.cmd_port))
        except OSError:
            pass
        self.rx_sock.close()
        self.tx_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HoverboardUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
