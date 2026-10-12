#!/usr/bin/env python3
"""Keyboard driving for the hoverboard robot (replaces yahboom_keyboard).

    ros2 run hoverboard_ctrl hoverboard_keyboard

Publishes cmd_vel continuously (10 Hz) while a direction is held, so the
bridge's 0.5 s command timeout never fires mid-drive; zero when idle.
A differential drive cannot strafe, so Yahboom's x/y switch ('t') is gone.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

HELP = """
Hoverboard keyboard control
---------------------------
   u    i    o
   j    k    l        k / space : stop
   m    ,    .
q/z : all speeds +/-10%    w/x : linear only    e/c : angular only
b   : buzzer on/off        s   : pause/resume keyboard control
CTRL-C to quit
"""

DEFAULT_TOPICS = None   # set by hoverboard_multi's multi_keyboard_ctrl

MOVE = {'i': (1, 0), 'o': (1, -1), 'j': (0, 1), 'l': (0, -1), 'u': (1, 1),
        ',': (-1, 0), '.': (-1, 1), 'm': (-1, -1)}
SPEED = {'q': (1.1, 1.1), 'z': (.9, .9), 'w': (1.1, 1), 'x': (.9, 1), 'e': (1, 1.1), 'c': (1, .9)}


class HoverboardKeyboard(Node):
    def __init__(self):
        super().__init__('keyboard_ctrl')
        self.v_max = float(self.declare_parameter('linear_speed_limit', 0.5).value)
        self.w_max = float(self.declare_parameter('angular_speed_limit', 1.5).value)
        topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel').value
        # Several robots at once (hoverboard_multi): the same command to each.
        topics = list(self.declare_parameter('cmd_vel_topics', DEFAULT_TOPICS or [topic]).value)
        self.pubs = [self.create_publisher(Twist, t, 10) for t in topics]
        print('cmd_vel -> ' + ', '.join(topics))

    def publish(self, t):
        for p in self.pubs:
            p.publish(t)
        self.pub_buzzer = self.create_publisher(Bool, 'Buzzer', 1)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = HoverboardKeyboard()
    speed, turn = 0.2, 0.8
    x = th = 0
    idle = 0
    paused = False
    buzzer = False
    print(HELP)
    print(f'speed {speed:.2f} m/s  turn {turn:.2f} rad/s')
    try:
        while rclpy.ok():
            key = get_key(settings).lower()
            if key == '\x03':
                break
            if key in MOVE:
                x, th = MOVE[key]
                idle = 0
            elif key in SPEED:
                speed = min(node.v_max, speed * SPEED[key][0])
                turn = min(node.w_max, turn * SPEED[key][1])
                print(f'speed {speed:.2f} m/s  turn {turn:.2f} rad/s')
            elif key in (' ', 'k'):
                x = th = 0
            elif key == 's':
                paused = not paused
                print('keyboard control ' + ('PAUSED' if paused else 'resumed'))
            elif key == 'b':
                buzzer = not buzzer
                node.pub_buzzer.publish(Bool(data=buzzer))
            else:
                idle += 1
                if idle > 4:            # ~0.5 s after the key is released
                    x = th = 0
            t = Twist()
            if not paused:
                t.linear.x = speed * x
                t.angular.z = turn * th
            node.publish(t)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.publish(Twist())
        if buzzer:
            node.pub_buzzer.publish(Bool(data=False))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
