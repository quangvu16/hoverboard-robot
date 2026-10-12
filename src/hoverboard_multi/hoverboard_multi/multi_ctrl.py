"""Drive every robot with one keyboard or gamepad (same command to each).

    ros2 run hoverboard_multi multi_keyboard_ctrl                  # robot1, robot2
    ros2 run hoverboard_multi multi_keyboard_ctrl --ros-args -p cmd_vel_topics:="['/robot1/cmd_vel','/robot2/cmd_vel','/robot3/cmd_vel']"
    ros2 launch hoverboard_multi multi_joy_launch.py

These are hoverboard_ctrl's keyboard and gamepad nodes pointed at several
cmd_vel topics - one implementation instead of Yahboom's copies.
"""
DEFAULT = ['/robot1/cmd_vel', '/robot2/cmd_vel']


def keyboard():
    from hoverboard_ctrl import hoverboard_keyboard
    hoverboard_keyboard.DEFAULT_TOPICS = DEFAULT
    hoverboard_keyboard.main()
