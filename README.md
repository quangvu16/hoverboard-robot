# hoverboard-robot

Hoverboard + LiDAR + ESP32 + ROS 2 Humble = an autonomous mobile robot.

The motors, mainboard and battery come from a scrapped self-balancing hoverboard.
An ESP32 reads the LiDAR, IMU and sonars, drives the mainboard, and streams everything
over Wi-Fi to a ROS 2 Humble PC. The PC runs SLAM (Cartographer or slam_toolbox), Nav2,
an EKF and RViz. You can also drive the robot from a web page or an Android app.

This repository is the PC-side colcon workspace (`~/robot_ws`). A full illustrated guide
is in [`website/index.html`](website/index.html): open it in a browser.

## Video

[![Hoverboard + ESP32 + LiDAR = navigation robot (YouTube)](https://img.youtube.com/vi/447QFAo13s0/hqdefault.jpg)](https://www.youtube.com/watch?v=447QFAo13s0)

Watch the robot on YouTube: [Hoverboard + ESP32 + LiDAR = Navigation Robot](https://www.youtube.com/watch?v=447QFAo13s0).

## How it works

```
 ON THE CHASSIS                                     ROS 2 PC (Wi-Fi, UDP)
 ┌──────────────────┐ UART  ┌──────────────┐ LDS1 :9900 ┌─────────────────────────┐
 │ LD14P LiDAR      ├──────►│              ├───────────►│ udp_lidar_receiver_node │──► /scan
 ├──────────────────┤ UART  │              │ HBT1 :9902 ├─────────────────────────┤
 │ Hoverboard board │◄─────►│    ESP32     ├───────────►│ hoverboard_udp_bridge   │──► /odom, /imu/data,
 │ (FOC firmware)   │       │              │◄───────────┤                         │    /ultrasonic/*, /lights/*
 ├──────────────────┤ I²C   │ speed loop,  │ HBC1 :9901 └───────────┬─────────────┘◄── /cmd_vel
 │ MPU9250 IMU      ├──────►│ IMU fusion,  │                        ▼
 ├──────────────────┤ GPIO  │ sonar reflex │            robot_localization EKF ──► /odom/filtered
 │ 3× HC-SR04, LEDs │◄─────►│              │                        ▼
 └──────────────────┘       └──────────────┘            Cartographer / slam_toolbox + Nav2
```

The work is split cleanly. Anything that must be on time, or keep working with Wi-Fi
down, runs on the ESP32: the wheel speed loop, sonar reflexes, lights and IMU fusion.
Anything that needs real compute runs on the PC: maps, plans, costmaps and the EKF.
The ESP32 only knows RPM and ticks. All geometry in metres lives in one file,
`src/hoverboard_base_node/config/hoverboard_base.yaml`.

| Datagram | Port | Direction | Contents |
|---|---|---|---|
| `LDS1` | UDP 9900 | ESP32 → PC | 360 LiDAR ranges in mm |
| `HBT1` | UDP 9902 | ESP32 → PC | wheel ticks, RPM, battery, sonars, lights, IMU (74 bytes) |
| `HBC1` | UDP 9901 | PC → ESP32 | wheel RPM targets, buzzer |

## Hardware

| Part | Notes |
|---|---|
| Hoverboard mainboard + 2 hub motors | [hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC), `VARIANT_USART`. Wheel radius 96.15 mm, 90 ticks/rev |
| 36 V hoverboard battery | Powers the mainboard and motors |
| ESP32-WROOM DevKit | The only processor on the robot |
| LDROBOT LD14P LiDAR | 360° scan, ~6 Hz |
| MPU9250 IMU | Gyro + accelerometer on I²C (SDA 21, SCL 22). No compass: the hub-motor magnets corrupt it |
| 3× HC-SR04 | Front, left, right. The ESP32 backs off below 350 mm (front) and turns away below 50 mm (sides) |
| LEDs + buzzer | Turn and stop lights follow the wheel motion; the buzzer is on `/Buzzer` |

Connect only GND, TX and RX between the ESP32 and the mainboard. Leave the mainboard's
VCC pin open: it can carry 15 V.

## ESP32 firmware

Two Arduino sketches live in [`firmware/`](firmware/). Both use only the ESP32 Arduino core,
with no third-party libraries:

| Sketch | IMU | Notes |
|---|---|---|
| [`ldrobot_ld14p_esp32_mpu9250_ros`](firmware/ldrobot_ld14p_esp32_mpu9250_ros) | MPU9250 | **In use.** Gyro + accel fusion on the ESP32 |
| [`ldrobot_ld14p_esp32_ros`](firmware/ldrobot_ld14p_esp32_ros) | BNO055 | The original. Its [`HOVERBOARD.md`](firmware/ldrobot_ld14p_esp32_ros/HOVERBOARD.md) is the full build and wiring guide |

All pins, ports and the PC's address (`DEST_IP`) are set in each sketch's `config.h`.
Wi-Fi credentials are kept out of git: copy `secrets.h.example` to `secrets.h` in the
sketch folder and fill in your network. Then flash at 115200 baud (faster uploads fail on
this board):

```bash
S=~/robot_ws/firmware/ldrobot_ld14p_esp32_mpu9250_ros
cp $S/secrets.h.example $S/secrets.h   # then edit WIFI_SSID / WIFI_PASS
arduino-cli compile --fqbn esp32:esp32:esp32 $S
arduino-cli upload  --fqbn esp32:esp32:esp32:UploadSpeed=115200 -p /dev/ttyUSB0 $S
```

## Repository layout

| Path | What it is |
|---|---|
| `src/esp_tcp_server` | The translation layer: `hoverboard_udp_bridge_node`, `udp_lidar_receiver_node`, and the older TCP `esp_tcp_server_node` |
| `src/hoverboard_base_node` | `config/hoverboard_base.yaml`, the one place for wheel geometry and speed limits |
| `src/hoverboard_description` | `hoverboard.urdf` (chassis, LiDAR, IMU, sonar frames) |
| `src/robot_bringup` | `robot_bringup.launch.py` (base + LiDAR + URDF + EKF), `robot_slam.launch.py`, EKF config |
| `src/robot_cartographer` | Cartographer + Nav2 + RViz in one launch, `save_map`, `initialpose_relay` |
| `src/robot_navigation` | Nav2 parameters for this base (`param/hoverboard.yaml`) |
| `src/robot_web_nav` | Web page for navigation over rosbridge |
| `src/hoverboard_*` | Ports of the Yahboom packages: gamepad, laser behaviours, camera, multi-robot. See [`src/HOVERBOARD_PACKAGES.md`](src/HOVERBOARD_PACKAGES.md) |
| `src/auto_mapper`, `src/m-explore-ros2`, `src/full_coverage_path_planner` | Vendored frontier explorers and a coverage planner |
| `firmware/` | The ESP32 Arduino sketches (not built by colcon) |
| `android_app/` | Robot Nav, a Kotlin Android app (see its [README](android_app/README.md)) |
| `tools/` | Calibration scripts and the systemd autostart files |
| `website/` | The project's web page |

slam_toolbox is used from a separate workspace (`~/slam_toolbox_ws`, v2.6.9 retuned for
this robot) and is not in this repository.

## Install

On Ubuntu 22.04:

```bash
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-cartographer-ros ros-humble-robot-localization \
  ros-humble-rmw-cyclonedds-cpp ros-humble-teleop-twist-keyboard \
  ros-humble-rosbridge-suite python3-colcon-common-extensions

git clone https://github.com/quangvu16/hoverboard-robot.git ~/robot_ws
cd ~/robot_ws && colcon build
```

Add to `~/.bashrc`. Source `local_setup.bash`, not `setup.bash`: the latter can pull in
stale overlays recorded at build time.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Builds are plain `colcon build` (no `--symlink-install`), and config is read from the
install space. After editing a YAML file, rebuild that package.

## Run

The PC and the ESP32 must be on the same network. If ufw is enabled:
`sudo ufw allow 9900:9902/udp`.

```bash
# Bring up the robot: bridge, LiDAR, URDF, EKF
ros2 launch robot_bringup robot_bringup.launch.py
ros2 launch robot_bringup robot_bringup.launch.py ekf:=false   # raw wheel odometry only

# Check every link
ros2 topic echo /hoverboard/link_status
ros2 topic hz /scan        # ~6 Hz
ros2 topic hz /odom        # ~50 Hz

# Map with Cartographer (Nav2 and RViz included), then save
ros2 launch robot_cartographer cartographer_nav.launch.py
ros2 run robot_cartographer save_map ~/m_map

# Navigate on the saved map
ros2 launch robot_cartographer cartographer_nav.launch.py map:=$HOME/m_map.pbstream

# Or: slam_toolbox, then Nav2 + AMCL
ros2 launch robot_bringup robot_slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/m_map
ros2 launch robot_navigation navigation2.launch.py map:=$HOME/m_map.yaml

# Drive by hand
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch hoverboard_ctrl joy_launch.py
```

Run one stack at a time: two bringups fight over UDP 9902, and Cartographer and
slam_toolbox must never run together. For LiDAR + IMU only mapping, pass
`wheel_odom:=false` to **both** bringup and `cartographer_nav`.

Lift the wheels off the ground for the first drive: a hoverboard has the torque to
throw the chassis.

## Web page and Android app

```bash
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream
ros2 launch robot_web_nav web_nav.launch.py stack:=amcl map:=$HOME/m_map.yaml
```

Open `http://<robot-pc-ip>:8000` on any device on the same Wi-Fi. Click a spot to send
the robot there, drag to set its final heading, set its pose, cancel, or save named spots.
The Android app in `android_app/` uses the same rosbridge websocket (`ws://<robot-pc-ip>:9090`).

## Start at boot

`tools/robot_bringup.sh` and `tools/robot-bringup.service` run the bringup as a systemd
service:

```bash
sudo cp ~/robot_ws/tools/robot-bringup.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now robot-bringup
journalctl -u robot-bringup -f
```

The unit runs as user `vu` from `/home/vu/robot_ws`; edit `User=` and `ExecStart=` for
your machine.

## Calibration

Once the LiDAR and IMU are fixed to the chassis:

```bash
python3 ~/robot_ws/tools/lidar_mirror_test.py        # is /scan mirrored or rotated?
python3 ~/robot_ws/tools/calib_wheel_separation.py   # odom yaw vs Cartographer yaw
```

## Status

- **Verified:** wheel radius (odometry read 0.61 m for a real 0.60 m); the ESP32 speed
  loop holds 30 rpm and drives 0.6 m straight at 0.2 m/s.
- **Working:** bridge, LiDAR, MPU9250, EKF, and the Cartographer and Nav2 launch files.
- **To do:** wheel separation (0.18 m) is unverified until the sensors are mounted on the
  chassis. The web page and Android app are built but not yet tested against a live robot.

## Learn more

- [ROS 2 Humble docs](https://docs.ros.org/en/humble/) · [Nav2](https://docs.nav2.org/) ·
  [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/) ·
  [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) ·
  [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- [hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC) ·
  [Arduino core for ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/) ·
  [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

## License

MIT, see [LICENSE](LICENSE). Vendored packages in `src/` keep their own licences.
