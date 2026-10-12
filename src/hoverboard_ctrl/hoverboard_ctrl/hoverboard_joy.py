#!/usr/bin/env python3
"""Gamepad driving for the hoverboard robot (replaces Yahboom's yahboom_joy).

    ros2 launch hoverboard_ctrl joy_launch.py

Subscribes  joy (sensor_msgs/Joy, from joy_node)
Publishes   cmd_vel     geometry_msgs/Twist   only while in MANUAL mode
            JoyState    std_msgs/Bool         true = manual; the hoverboard_laser
                                              and vision nodes pause while true
            Buzzer      std_msgs/Bool         toggled by the buzzer button;
                                              hoverboard_udp_bridge_node drives
                                              the ESP32 buzzer from it
            servo_s1/servo_s2 std_msgs/Int32  only if enable_servos is true
                                              (camera pan/tilt, not fitted now)

Mode button: toggles MANUAL/AUTO. Entering manual also cancels any Nav2 goal
(the navigate_to_pose action) and sends a stop, so the gamepad always wins.
Gear buttons cycle the linear (1/3, 2/3, 1) and angular (1/4 .. 1) gears.

Axis/button numbers are parameters because every gamepad numbers them
differently: run `ros2 topic echo /joy` and press things to find yours.
"""
import time

import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32

LINEAR_GEARS = [1.0 / 3, 2.0 / 3, 1.0]
ANGULAR_GEARS = [0.25, 0.5, 0.75, 1.0]


class HoverboardJoy(Node):

    def __init__(self):
        super().__init__('joy_ctrl')
        p = self.declare_parameter
        # Limits sized for a hoverboard base: the bridge caps a wheel at 0.5 m/s.
        self.v_max = float(p('xspeed_limit', 0.5).value)
        self.w_max = float(p('angular_speed_limit', 1.5).value)
        self.axis_linear = int(p('axis_linear', 1).value)
        self.axis_angular = int(p('axis_angular', 2).value)
        self.deadzone = float(p('deadzone', 0.2).value)
        self.btn_mode = int(p('button_mode', 7).value)
        self.btn_buzzer = int(p('button_buzzer', 11).value)
        self.btn_gear_linear = int(p('button_gear_linear', 9).value)
        self.btn_gear_angular = int(p('button_gear_angular', 10).value)
        self.start_manual = bool(p('start_manual', True).value)
        self.enable_servos = bool(p('enable_servos', False).value)
        self.btn_servo = [int(x) for x in p('buttons_servo_up_down_left_right', [1, 3, 0, 4]).value]
        cmd_topic = p('cmd_vel_topic', 'cmd_vel').value
        # Several robots at once (hoverboard_multi): the same command to each.
        topics = list(p('cmd_vel_topics', [cmd_topic]).value)

        self.manual = self.start_manual
        self.buzzer = False
        self.gear_l = 0
        self.gear_a = 0
        self.pan, self.tilt = 0, -60
        self.prev_buttons = []
        self.last_mode_toggle = 0.0

        self.pubs_cmd = [self.create_publisher(Twist, t, 10) for t in topics]
        self.pub_state = self.create_publisher(Bool, 'JoyState', 10)
        self.pub_buzzer = self.create_publisher(Bool, 'Buzzer', 1)
        if self.enable_servos:
            self.pub_s1 = self.create_publisher(Int32, 'servo_s1', 10)
            self.pub_s2 = self.create_publisher(Int32, 'servo_s2', 10)
        self.cancel_cli = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.get_logger().info('cmd_vel -> ' + ', '.join(topics))
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            f'{"MANUAL" if self.manual else "AUTO"} mode; limits {self.v_max} m/s, '
            f'{self.w_max} rad/s; mode button {self.btn_mode}, buzzer {self.btn_buzzer}')

    def publish_cmd(self, t):
        for pub in self.pubs_cmd:
            pub.publish(t)

    def publish_state(self):
        self.pub_state.publish(Bool(data=self.manual))

    def pressed(self, buttons, i):
        """Rising edge, so holding a button toggles once."""
        if i < 0 or i >= len(buttons):
            return False
        was = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
        return buttons[i] == 1 and was == 0

    def axis(self, joy, i):
        if i < 0 or i >= len(joy.axes):
            return 0.0
        v = joy.axes[i]
        return 0.0 if abs(v) < self.deadzone else v

    def on_joy(self, joy):
        b = list(joy.buttons)
        if self.pressed(b, self.btn_mode) and time.time() - self.last_mode_toggle > 0.5:
            self.last_mode_toggle = time.time()
            self.manual = not self.manual
            self.publish_state()
            self.publish_cmd(Twist())
            if self.manual:
                self.cancel_navigation()
            self.get_logger().info('MANUAL mode' if self.manual else 'AUTO mode')
        if self.pressed(b, self.btn_buzzer):
            self.buzzer = not self.buzzer
            self.pub_buzzer.publish(Bool(data=self.buzzer))
        if self.pressed(b, self.btn_gear_linear):
            self.gear_l = (self.gear_l + 1) % len(LINEAR_GEARS)
            self.get_logger().info(f'linear gear {LINEAR_GEARS[self.gear_l]:.2f}')
        if self.pressed(b, self.btn_gear_angular):
            self.gear_a = (self.gear_a + 1) % len(ANGULAR_GEARS)
            self.get_logger().info(f'angular gear {ANGULAR_GEARS[self.gear_a]:.2f}')
        if self.enable_servos:
            self.servos(b)
        self.prev_buttons = b

        if self.manual:
            t = Twist()
            t.linear.x = max(-self.v_max, min(self.v_max,
                             self.axis(joy, self.axis_linear) * self.v_max * LINEAR_GEARS[self.gear_l]))
            t.angular.z = max(-self.w_max, min(self.w_max,
                              self.axis(joy, self.axis_angular) * self.w_max * ANGULAR_GEARS[self.gear_a]))
            self.publish_cmd(t)

    def servos(self, b):
        up, down, left, right = self.btn_servo
        changed = False
        if up < len(b) and b[up]:
            self.pan = min(90, self.pan + 1); changed = True
        if down < len(b) and b[down]:
            self.pan = max(-90, self.pan - 1); changed = True
        if left < len(b) and b[left]:
            self.tilt = max(-90, self.tilt - 1); changed = True
        if right < len(b) and b[right]:
            self.tilt = min(20, self.tilt + 1); changed = True
        if changed:
            self.pub_s1.publish(Int32(data=self.pan))
            self.pub_s2.publish(Int32(data=self.tilt))

    def cancel_navigation(self):
        # An all-zero goal id with a zero stamp cancels every goal on the server.
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())
            self.get_logger().info('cancelled Nav2 goals')


def main():
    rclpy.init()
    node = HoverboardJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish_cmd(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
