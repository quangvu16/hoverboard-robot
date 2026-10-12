# Hoverboard packages (ported from Yahboom)

The `yahboom*` packages in this folder were rewritten for the hoverboard
robot as `hoverboard_*` packages. The originals are kept for reference and
carry a `COLCON_IGNORE`, so they are no longer built.

The robot underneath: hoverboard mainboard + ESP32 (firmware
`~/Arduino/ldrobot_ld14p_esp32_ros`, see its `HOVERBOARD.md`), talking UDP to
`hoverboard_udp_bridge_node` (`esp_tcp_server`). Topics the ports rely on:
`/cmd_vel`, `/odom`, `/scan`, `/JoyState`, `/Buzzer` (std_msgs/Bool), and for
the vision nodes `/camera/image_raw/compressed` + `/image_raw`.

| Yahboom package | Now | What changed |
|---|---|---|
| yahboomcar_base_node | **hoverboard_base_node** | The odom_raw→TF node is gone (the bridge does it). Holds `config/hoverboard_base.yaml`, the ONE place for wheel radius / separation / limits, used by `robot_bringup` too. |
| yahboomcar_description | **hoverboard_description** | One URDF (`hoverboard.urdf`): 0.33 m chassis, 96 mm wheels, `base_scan`, `sonar_link`. Multi-robot via frame prefix instead of a URDF copy per robot. |
| yahboomcar_ctrl | **hoverboard_ctrl** | `hoverboard_joy` / `hoverboard_keyboard`. Hoverboard speed limits, gamepad mapping in `config/joy.yaml`, buzzer on `/Buzzer`, mode switch cancels Nav2 goals, can drive several robots (`cmd_vel_topics`). No strafing. |
| yahboomcar_laser | **hoverboard_laser** | Avoidance / tracker / warning. **Fixed:** the front-left sector was never checked with this LiDAR's negative-increment scans; inf/NaN and the robot's own body (<0.30 m) filtered; no `sleep()` in callbacks; speeds bounded. |
| yahboomcar_nav | **hoverboard_nav** | Cartographer mapping reuses `robot_cartographer`; gmapping without the fake laser TF; navigation on `robot_navigation/param/hoverboard.yaml`; maps saved to `~/maps`. **Fixed:** `app_send_goal` republished its own goals forever. |
| yahboomcar_multi | **hoverboard_multi** | Per-robot bringup (own UDP ports, namespace, prefixed frames), per-robot Nav2 with params generated from hoverboard.yaml, TF relay, differential-drive formation follower (Yahboom's steered sideways). |
| yahboomcar_msgs | **hoverboard_msgs** | Unchanged messages. |
| yahboom_web_savmap_interfaces | **hoverboard_web_savmap_interfaces** | Unchanged service. |
| yahboom_app_save_map | **hoverboard_app_save_map** | Paths → `~/maps`, catalogue in `~/.ros`; keeps the app's service name. **Fixed:** crashed on duplicate names; client called a non-existent service. |
| yahboom_esp32_camera | **hoverboard_camera** | New `camera_source` (USB camera or ESP32-CAM MJPEG URL). Patrol fixed (scan angles, buzzer). Odometry calibration rewritten to output `wheel_radius_m` / `wheel_separation_m`. |
| yahboom_esp32ai_car | **hoverboard_ai_car** | Camera topic, buzzer, data paths (`~/.ros/hoverboard_ai_car/colorHSV.text`). **Fixed:** crashes in `media_library` (typo, ROS1 `unregister`, wrong arg count). |
| yahboom_esp32_mediapipe | **hoverboard_mediapipe** | Camera topic, data paths. **Fixed:** face detector loaded the eye model. |

Yahboom's IMU filter, EKF, MS200 LiDAR, Mcnamu driver and micro-ROS agent have
no hoverboard equivalent and were dropped.

## Quick start

```bash
ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard   # base + LiDAR + URDF

ros2 launch hoverboard_ctrl joy_launch.py                  # gamepad
ros2 run hoverboard_ctrl hoverboard_keyboard               # keyboard

ros2 run hoverboard_laser laser_Avoidance                  # wander, avoid obstacles
ros2 run hoverboard_laser laser_Tracker                    # follow nearest object
ros2 run hoverboard_laser laser_Warning                    # face it, buzz when close

ros2 launch hoverboard_nav map_cartographer_launch.py      # map
ros2 launch hoverboard_nav save_map_launch.py map_name:=home
ros2 launch hoverboard_nav navigation_launch.py map:=$HOME/maps/home.yaml
ros2 launch robot_cartographer cartographer_nav.launch.py  # navigate while mapping

ros2 run hoverboard_camera calibrate_linear                # odometry calibration
ros2 run hoverboard_camera calibrate_angular
ros2 run hoverboard_camera patrol                          # square/triangle/circle

ros2 launch hoverboard_camera camera_launch.py source:=0   # vision (needs a camera)
ros2 run hoverboard_ai_car colorTracker
```

Pause any autonomous node by switching the gamepad to manual (its mode button
publishes `/JoyState` true). Several also take `ros2 param set <node> Switch true`.

## Multi-robot

Each robot's ESP32 needs its own UDP ports (`DEST_PORT`, `HB_CMD_PORT`,
`HB_TELEM_PORT` in that robot's firmware `config.h`):

```bash
ros2 launch hoverboard_multi robot_bringup_multi.launch.py robot_name:=robot1
ros2 launch hoverboard_multi robot_bringup_multi.launch.py robot_name:=robot2 scan_port:=9910 cmd_port:=9911 telem_port:=9912
ros2 launch hoverboard_multi robot_nav_multi.launch.py robot_name:=robot1 map:=$HOME/maps/home.yaml
ros2 launch hoverboard_multi robot_nav_multi.launch.py robot_name:=robot2 map:=$HOME/maps/home.yaml
ros2 launch hoverboard_multi formation_launch.py followers:=robot2
ros2 run hoverboard_multi multi_keyboard_ctrl
```

## Not installed yet / not tested on the robot

- `pip install mediapipe` — the hand/gesture/pose/face nodes (`hoverboard_mediapipe`,
  and `hoverboard_ai_car` HandCtrl / FingerCtrl / RobotCtrl / control_shape).
- `sudo apt install python3-pyzbar` (or `pip install pyzbar`) — `qrTracker`.
- `pip install dlib` — `06_FaceLandmarks`.
- `sudo apt install ros-humble-rosbridge-server` — the `*_app_launch.xml` files (Yahboom phone/web app).
- No camera is fitted: vision nodes were only run against a synthetic video.
- Multi-robot was tested with one real robot and simulated TF only.
- Buzzer: firmware drives **GPIO25** (active buzzer, `config.h`); wire it and test with
  `ros2 topic pub --once /Buzzer std_msgs/msg/Bool "data: true"` (then `false`).
