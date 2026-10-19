# hoverboard-robot

Hoverboard + LiDAR + ESP32 + ROS 2 Humble = an autonomous mobile robot.

The motors, mainboard and battery come from a scrapped self-balancing hoverboard.
An ESP32 reads the LiDAR, IMU and sonars, drives the mainboard, and streams everything
over Wi-Fi to a ROS 2 Humble PC. The PC runs SLAM (Cartographer or slam_toolbox), Nav2,
an EKF and RViz. You can also drive the robot from a web page or an Android app, and it
docks itself to charge when the battery runs low.

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
 │ Hoverboard board │◄─────►│    ESP32     ├───────────►│ hoverboard_udp_bridge   │──► /odom, /imu/data, /range/*,
 │ (FOC firmware)   │       │              │◄───────────┤                         │    /ultrasonic/*, /lights/*, battery
 ├──────────────────┤ I²C   │ speed loop,  │ HBC1 :9901 └───────────┬─────────────┘◄── /cmd_vel_mux (safety chain below)
 │ MPU9250 IMU      ├──────►│ IMU fusion,  │                        ▼
 ├──────────────────┤ GPIO  │ sonar reflex,│            robot_localization EKF ──► /odom/filtered
 │ 3× HC-SR04, LEDs │◄─────►│ battery stop │                        ▼
 └──────────────────┘       └──────────────┘            Cartographer / slam_toolbox + Nav2
```

The work is split cleanly. Anything that must be on time, or keep working with Wi-Fi
down, runs on the ESP32: the wheel speed loop, sonar reflexes, low-battery stop, lights and
IMU fusion.
Anything that needs real compute runs on the PC: maps, plans, costmaps and the EKF.
The ESP32 only knows RPM and ticks. All geometry in metres lives in one file,
`src/hoverboard_base_node/config/hoverboard_base.yaml`.

| Datagram | Port | Direction | Contents |
|---|---|---|---|
| `LDS1` | UDP 9900 | ESP32 → PC | 360 LiDAR ranges in mm |
| `HBT1` | UDP 9902 | ESP32 → PC | wheel ticks, RPM, battery, sonars, lights, IMU, battery state (78 bytes) |
| `HBC1` | UDP 9901 | PC → ESP32 | wheel RPM targets, buzzer |

## Hardware

| Part | Notes |
|---|---|
| Hoverboard mainboard + 2 hub motors | [hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC), `VARIANT_USART`. Wheel radius 96.15 mm, 90 ticks/rev |
| 36 V hoverboard battery | Powers the mainboard and motors |
| ESP32-WROOM DevKit, or ESP32-S3-DevKitC-1 v1.1 | The only processor on the robot (pick the matching sketch below) |
| LDROBOT LD14P LiDAR | 360° scan, ~6 Hz |
| MPU9250 IMU | Gyro + accelerometer on I²C (SDA 21, SCL 22). No compass: the hub-motor magnets corrupt it |
| 3× HC-SR04 | Front, left, right. The ESP32 backs off below 350 mm (front) and turns away below 50 mm (sides) |
| LEDs + buzzer | Turn and stop lights follow the wheel motion; the buzzer is on `/Buzzer` |

Connect only GND, TX and RX between the ESP32 and the mainboard. Leave the mainboard's
VCC pin open: it can carry 15 V.

## ESP32 firmware

Three Arduino sketches live in [`firmware/`](firmware/). Both use only the ESP32 Arduino core,
with no third-party libraries:

| Sketch | Board | IMU | Notes |
|---|---|---|---|
| [`ldrobot_ld14p_esp32_mpu9250_ros`](firmware/ldrobot_ld14p_esp32_mpu9250_ros) | ESP32-WROOM | MPU9250 | **In use.** Gyro + accel fusion on the ESP32 |
| [`ldrobot_ld14p_esp32s3_mpu9250_ros`](firmware/ldrobot_ld14p_esp32s3_mpu9250_ros) | ESP32-S3-DevKitC-1 v1.1 | MPU9250 | Same firmware, new pin map (below) |
| [`ldrobot_ld14p_esp32_ros`](firmware/ldrobot_ld14p_esp32_ros) | ESP32-WROOM | BNO055 | The original. Its [`HOVERBOARD.md`](firmware/ldrobot_ld14p_esp32_ros/HOVERBOARD.md) is the full build and wiring guide |

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

### Low-battery protection

Both MPU9250 sketches watch the battery voltage the mainboard reports (`battery.h`), so the
robot stops on purpose before the pack is ruined or the mainboard cuts power:

| State | Filtered voltage (10S pack) | What the ESP32 does |
|---|---|---|
| ok | above 35.5 V | nothing |
| low | below 35.5 V (3.55 V/cell) | halves every wheel command and chirps every 30 s |
| critical | below 34.5 V (3.45 V/cell) | holds the wheels at zero, whatever ROS or the web stick asks, and double-chirps every 5 s |

- The voltage is low-pass filtered (5 s) and must stay past a threshold for 10 s, so a hard
  launch that sags the pack doesn't trip it.
- Going back to ok needs 37.0 V, which in practice means charging or a fresh pack. A pack
  that recovers a little at rest doesn't restart the robot.
- Critical sits above the mainboard's own power-off level (`BAT_DEAD`, 3.37 V/cell
  upstream), which cuts power without warning.
- The thresholds, cell count, speed factor and beeps are in the battery section of
  `config.h`. They trust the mainboard's voltage reading, so calibrate its `BAT_CALIB_*` first.
- ROS sees the state on `/hoverboard/battery_state` (`ok` / `low` / `critical` / `unknown`,
  latched) and in `/hoverboard/link_status`. The bridge logs every change. The ESP32 drive
  page shows it too.

### Drive from a phone, no PC needed

Both MPU9250 sketches serve their own drive page. Open `http://<esp32-ip>/` or
`http://hoverboard.local/` on a phone on the same Wi-Fi (the Serial Monitor prints the IP at
boot), and drive with the stick, the RPM sliders or `W` `A` `S` `D`.

- The page talks to the board over a WebSocket on `ws://<esp32-ip>/ws`. It sends
  `D <rpm_left> <rpm_right>` ten times a second while you hold the stick, and `S` to stop.
  The board answers with a JSON status five times a second: battery, wheel RPM, the three
  sonars, the reflex, and who has the wheels.
- Commands take the same path as ROS's HBC1, so the sonar reflex and the mainboard timeout
  still apply. Each wheel is capped at `WEB_DRIVE_MAX_RPM` (60 rpm).
- While you hold the stick it overrides ROS. 0.5 s after the last web command, the wheels
  are zeroed and ROS gets them back. Cancel a Nav2 goal first if you don't want it to resume.
- Closing the page, switching apps or losing Wi-Fi stops the robot.
- Anyone on the Wi-Fi can open the page. Set `WEB_DRIVE_ENABLED 0` in `config.h` to turn it
  off. It is written with the ESP32 core only (WiFi, ESPmDNS, mbedtls), so the sketches
  still need no third-party libraries.

### Firmware updates over Wi-Fi

After one USB flash with a password, both MPU9250 sketches update over Wi-Fi (`ota.h`):

```bash
# once: add the password to the sketch's secrets.h, then flash over USB as above
echo '#define OTA_PASSWORD "choose-a-long-password"' >> firmware/ldrobot_ld14p_esp32_mpu9250_ros/secrets.h

# every update after that: build + upload over Wi-Fi
tools/ota_upload.sh                                                   # WROOM sketch, hoverboard.local
tools/ota_upload.sh firmware/ldrobot_ld14p_esp32s3_mpu9250_ros 192.168.1.50   # S3, by IP
```

- **No password, no OTA.** Without `OTA_PASSWORD` in `secrets.h`, OTA stays off,
  because anyone on the Wi-Fi could otherwise reflash the robot. The script reads the
  password from the same file, so the new image keeps OTA on.
- **Only while standing still.** The board ignores update requests while the wheels are
  commanded or turning; `espota` then reports *No response from the ESP*. Stop the robot
  and run it again.
- **Wheels held at zero** from the first byte until the reboot, whatever ROS, the web stick
  or the sonar reflex asks. LiDAR scans pause for the 10-30 s of the upload.
- **A bad image doesn't brick it.** The new firmware goes into the second app slot of the
  default partition scheme (1.25 MB each; the sketch is about 0.85 MB). If it doesn't
  boot, the old one does.
- **Firewall.** The board connects back to the PC on TCP 38232 (`OTA_HOST_PORT`). With ufw
  on, run `sudo ufw allow 38232/tcp`.
- **Names.** `hoverboard.local` is announced over mDNS (`ROBOT_HOSTNAME` in `config.h`), the
  same name as the drive page. If it doesn't resolve, use the IP the Serial Monitor prints.

### ESP32-S3-DevKitC-1 v1.1

The S3 build follows Espressif's
[v1.1 user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html).
The S3 has no GPIO 22-25, its flash and octal PSRAM take 26-37, and v1.1 has the RGB LED
on GPIO 38. So these signals moved (`config.h` refuses a bad pin at compile time). The full side-by-side pinout, with header
drawings of both boards, is in [`firmware/PINOUT.md`](firmware/PINOUT.md):

| Signal | ESP32-WROOM | ESP32-S3 | Header |
|---|---|---|---|
| Mainboard RX / TX | 26 / 27 | 6 / 7 | J1-6 / J1-7 |
| Buzzer | 25 | 21 | J3-18 |
| Front HC-SR04 TRIG / ECHO | 32 / 33 | 1 / 2 | J3-4 / J3-5 |
| Left HC-SR04 ECHO | 19 | 10 | J1-16 |
| Right HC-SR04 ECHO | 23 | 11 | J1-17 |
| MPU9250 SDA / SCL | 21 / 22 | 8 / 9 | J1-12 / J1-15 |
| Unchanged | LiDAR 16, LEDs 5 / 13 / 14, side TRIGs 17 / 18 | | |

Plug into the **UART** micro-USB port (it shows up as `/dev/ttyUSB0`), not the native USB port:

```bash
S=~/robot_ws/firmware/ldrobot_ld14p_esp32s3_mpu9250_ros
cp $S/secrets.h.example $S/secrets.h   # then edit WIFI_SSID / WIFI_PASS
arduino-cli compile --fqbn esp32:esp32:esp32s3 $S
arduino-cli upload  --fqbn esp32:esp32:esp32s3:UploadSpeed=115200 -p /dev/ttyUSB0 $S
```

The default board settings work on every DevKitC-1 variant. To use all of an
**ESP32-S3-N16R8**'s 16 MB flash and 8 MB PSRAM (the sketch doesn't need them), use
`--fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi` for both commands, adding
`,UploadSpeed=115200` on the upload. Of the S3's 45 GPIOs, the N16R8 leaves 24 safe
for wiring; see [`firmware/PINOUT.md`](firmware/PINOUT.md#how-many-gpios-the-s3-really-has).

If the upload doesn't start, hold **Boot**, tap **Reset**, then release **Boot**.

## Repository layout

| Path | What it is |
|---|---|
| `src/esp_tcp_server` | The translation layer: `hoverboard_udp_bridge_node` (also the sonars as `/range/*` and the battery state), `udp_lidar_receiver_node`, and the older TCP `esp_tcp_server_node` |
| `src/hoverboard_base_node` | `config/hoverboard_base.yaml`, the one place for wheel geometry and speed limits |
| `src/hoverboard_description` | `hoverboard.urdf` (chassis, LiDAR, IMU, sonar frames) |
| `src/robot_bringup` | `robot_bringup.launch.py` (base + LiDAR + URDF + EKF + the [safety chain](#safety-chain-for-velocity-commands)), `robot_slam.launch.py`, EKF, collision monitor and `twist_mux` configs |
| `src/robot_cartographer` | Cartographer + Nav2 + RViz in one launch, `save_map`, `initialpose_relay` |
| `src/robot_navigation` | Nav2 parameters for this base (`param/hoverboard.yaml`, with the sonar range layer) |
| `src/robot_web_nav` | Web page for driving, navigation and docking over rosbridge |
| `src/robot_docking` | Auto-docking: dock saved on the map, Nav2 to the staging pose, reverse in; docks by itself when the battery is low |
| `src/hoverboard_*` | Ports of the Yahboom packages: gamepad, laser behaviours, camera, multi-robot. See [`src/HOVERBOARD_PACKAGES.md`](src/HOVERBOARD_PACKAGES.md) |
| `src/auto_mapper`, `src/m-explore-ros2`, `src/full_coverage_path_planner` | Vendored frontier explorers and a coverage planner |
| `firmware/` | The ESP32 Arduino sketches for the ESP32 and the ESP32-S3 (not built by colcon), and [`PINOUT.md`](firmware/PINOUT.md) |
| `android_app/` | Robot Nav, a Kotlin Android app (see its [README](android_app/README.md)) |
| `tools/` | Calibration scripts, the systemd autostart files and `ota_upload.sh` (firmware over Wi-Fi) |
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

### Safety chain for velocity commands

Bringup puts two arbiters between every velocity command and the wheels:

```
Nav2 --/cmd_vel--> collision_monitor --/cmd_vel_safe--> twist_mux --/cmd_vel_mux--> bridge --> ESP32
joystick, web stick, Android app, rqt --/cmd_vel_manual (beats Nav2)--^      ^
ros2 topic pub /cmd_vel_lock std_msgs/Bool "data: true"  (beats everything) -'
```

- **Collision monitor** (`src/robot_bringup/param/collision_monitor_hoverboard.yaml`). It
  slows Nav2 to 40 % when the LiDAR or a sonar sees something up to 0.60 m ahead of the
  centre, and stops it at 0.32 m. Manual driving doesn't pass through it; the ESP32 reflex
  still covers that. `collision_monitor:=false` turns it off.
- **twist_mux** (`src/robot_bringup/param/twist_mux_hoverboard.yaml`).
  - A stick on `/cmd_vel_manual` overrides Nav2 at once, and Nav2 gets the wheels back
    0.5 s after the last manual message.
  - `/cmd_vel_lock` `true` holds the robot still, joysticks included.
  - It needs `sudo apt install ros-humble-twist-mux`. Without it, bringup says so and falls
    back to the old "last message wins".
- **Sonars in Nav2.** The bridge publishes the three HC-SR04s as `sensor_msgs/Range` on
  `/range/{front,left,right}` (10 Hz). They feed:
  - the collision monitor;
  - a `RangeSensorLayer` in the local costmap, for glass, chair legs and low objects the
    LiDAR misses. It marks reliably from about 0.5 m; nearer, the collision monitor and the
    ESP32 reflex act on the raw ranges.
- **Update tf2 on this PC** (`sudo apt upgrade` gets `ros-humble-tf2-ros` 0.25.24).
  - Testing found a deadlock in tf2_ros 0.25.23, between Nav2's message filter and the TF
    listener. When it hits, the local costmap silently stops taking sensor data, LiDAR
    included.
  - 0.25.24 made it much rarer but not impossible. At the LiDAR's own rate it didn't occur
    in a 4-minute soak. Feeding the sonars as extra LaserScans triggered it within minutes,
    which is why they go in as Range messages, which don't use that filter.
  - The symptom: obstacles in RViz's local costmap stop updating while the robot moves.

Run one stack at a time: two bringups fight over UDP 9902, and Cartographer and
slam_toolbox must never run together. For LiDAR + IMU only mapping, pass
`wheel_odom:=false` to **both** bringup and `cartographer_nav`.

Lift the wheels off the ground for the first drive: a hoverboard has the torque to
throw the chassis.

## Web page and Android app

```bash
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream
ros2 launch robot_web_nav web_nav.launch.py stack:=amcl map:=$HOME/m_map.yaml
ros2 launch robot_web_nav web_nav.launch.py stack:=none      # only drive by hand
```

Open `http://<robot-pc-ip>:8000` on any device on the same Wi-Fi. It needs
`sudo apt install ros-humble-rosbridge-suite`. The Android app in `android_app/`
(`~/RobotNav.apk`) uses the same rosbridge websocket (`ws://<robot-pc-ip>:9090`), and so does
the drive panel on the project page ([`website/index.html`](website/index.html), chapter 09).
All three do the same things:

| Feature | Topics |
|---|---|
| **Drive stick** and `W` `A` `S` `D`: 10 commands a second while held, zeros on release; grabbing it cancels a Nav2 goal; leaving the page or app stops | `/cmd_vel_manual` (beats Nav2 in `twist_mux`) |
| **Hold still / Release**: holds the robot against Nav2 and every stick, and greys the stick. The label shows the lock set by any client | `/cmd_vel_lock` (needs `ros-humble-twist-mux`) |
| **Battery**: voltage and rough %; orange or red when the ESP32 is limiting speed (low) or holding the wheels (critical) | `/hoverboard/battery_state`, `/hoverboard/battery_volts`, `/battery_state` |
| **Sonar distances**: front, left and right in metres; `clear` = nothing within 2 m | `/range/{front,left,right}` |
| **Dock**: web page and app have Set dock here (asks first), Dock, Undock, Cancel, auto-dock on low battery, and the dock drawn on the map; the project page panel has Dock / Undock (Cancel while busy) | `/dock/command`, `/dock/status` (robot_docking) |
| **Navigation** (web page and app): click or tap a spot to go there, drag for the heading, set pose, cancel, saved spots | `/web/goal`, `/web/cancel`, `/initialpose`, `/web/nav_status` |

Without the PC, the ESP32's own [drive page](#drive-from-a-phone-no-pc-needed) at
`http://hoverboard.local/` drives the robot directly.

## Docking

`robot_docking` parks the robot in its dock, and does it by itself when the battery runs low.
`web_nav.launch.py` starts it (`docking:=true`, `auto_dock:=true`). It needs Nav2 running and
the robot localized on a map.

```
            dock (saved robot pose)          staging pose
   wall ▐ [robot] ──────── 0.7 m ──────────► ●
          ◄── 2. reverse in (dock_manager)   1. Nav2 drives here, facing away
```

1. **Set the dock once.** Park the robot in the dock, localized on the map, and press
   **Set dock here**. The pose is saved in `~/.ros/robot_dock.json` and survives restarts.
2. **Dock.**
   1. Nav2 drives forwards to a staging pose 0.7 m in front of the dock, facing away.
   2. The dock manager reverses the robot in, steering on the TF pose, and stops 1 cm short.
   3. If that takes over 40 s, it returns to the staging pose and tries once more.
3. **Undock** drives forwards 0.7 m, back to the staging pose.
4. **Auto-dock.** When the ESP32 reports battery `low` (below 35.5 V, speed already halved),
   the robot docks by itself and replaces any Nav2 goal. It doesn't wait for `critical`, because
   the ESP32 holds the wheels at zero then.

| Where | Controls |
|---|---|
| Web page `http://<pc>:8000`, Dock section | Set dock here, Dock, Undock, Cancel, auto-dock checkbox; the dock is drawn on the map |
| Android app, Dock section | the same |
| Project page panel ([`website/index.html`](website/index.html), chapter 09) | Dock / Undock (Cancel while busy) |
| Shell | `ros2 topic pub -1 /dock/command std_msgs/String "data: dock"` (also `save`, `undock`, `cancel`, `auto on`, `auto off`); `ros2 topic echo /dock/status` |

- **Why backwards.** The front sonar reverses the robot away from anything nearer than 350 mm,
  and the collision monitor only watches the front. Backing in keeps both pointed away from the
  dock.
- **Same safety chain.** The docking moves go through the same chain as Nav2's
  (`/cmd_vel` → collision monitor → `twist_mux`), so a stick or `/cmd_vel_lock` still overrides
  them.
- **Simulation results.** Requested and battery-low docking ended within 1–3 cm along the dock
  axis, 0–1 cm off it, and 1–4°.
- **Limits:**
  - With no camera, the final position is only as precise as the localization.
  - The mainboard measures motor current, not battery current, so charging isn't detected.
  - Charging needs contacts on the dock wired to the hoverboard's charge port.
- **Not Nav2's docking server.** Nav2's own server (`ros-humble-opennav-docking` 0.0.2) got
  backwards docking wrong in the same simulation, so a short reverse controller does the last
  0.7 m. It's worth revisiting with an AprilTag dock detector.
- **Troubleshooting:**
  - *Nav2 is not running* / *could not reach the staging pose*: start web_nav with a stack
    (`stack:=cartographer` or `amcl`) and set the pose first.
  - Docks beside or short of the charger: re-localize, or press Set dock here again with the
    robot in the dock.

Details: [`src/robot_docking/README.md`](src/robot_docking/README.md).

The bridge also publishes a standard `sensor_msgs/BatteryState` on `/battery_state`, at 1 Hz:

| Field | Value |
|---|---|
| `voltage` | the ESP32's filtered pack voltage |
| `percentage` | rough, from a resting Li-ion curve per cell (3.3 V = 0 %, 4.15 V = 100 %); reads low while driving |
| `power_supply_health` | `GOOD`, or `DEAD` at critical |
| `cell_voltage` | the pack voltage divided across the 10 cells |
| `current`, charging status | unknown: the mainboard measures the motors, not the pack |

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
- **Tested without the robot:** the safety chain, with a fake ESP32 and a fake LiDAR on
  an isolated ROS domain:
  - the wheel RPMs for slow-down, stop, manual over Nav2, lock and silence;
  - sonar marking and clearing in the costmap, and a 4-minute soak.
- **Tested without the robot:** the drive sticks, battery, sonar and lock in the web
  page, the app and the project page, against a fake rosbridge.
- **Tested without the robot:** docking, in a simulation with a fake ESP32 whose wheels follow
  the commands, and real Nav2:
  - save, undock, drive away, then dock (requested and on battery low) ended within 1–3 cm
    of the dock and 4°.
- **Tested without the robot:** the ESP32 drive page, low-battery protection and OTA. They
  compile for both boards and pass tests on the PC.
- **To do:**
  - flash the new firmware (`~/Arduino` still has the older sketch);
  - `sudo apt install ros-humble-twist-mux ros-humble-rosbridge-suite`;
  - `sudo apt upgrade` for tf2_ros 0.25.24 (see the [safety chain](#safety-chain-for-velocity-commands)).
- **To do:** wheel separation (0.18 m) is unverified until the sensors are mounted on the
  chassis. Then set the real sensor poses in `hoverboard.urdf` and re-check the collision
  monitor zones.

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
