# How to run every feature

A step-by-step guide, from a fresh install to every feature of the robot. The
[README](README.md) explains *how things work*; this file is only *what to type and press*.

Commands run on the **robot PC** (Ubuntu 22.04, ROS 2 Humble) unless the step says otherwise.
`<pc-ip>` is the robot PC's address on the Wi-Fi, and `<esp32-ip>` is the board's address,
printed on the Serial Monitor at boot.

| | Feature | Tested |
|---|---|---|
| 0 | [One-time setup](#0-one-time-setup) | |
| 1 | [Flash the ESP32](#1-flash-the-esp32) (USB, then over Wi-Fi) | compiles; OTA against a fake board |
| 2 | [Bring the robot up](#2-bring-the-robot-up) | on the robot |
| 3 | [Check every link](#3-check-every-link) | on the robot |
| 4 | [Drive by hand](#4-drive-by-hand) (PC, web, app, phone without PC) | simulation / browser |
| 5 | [Map a room](#5-map-a-room) (by hand or with Explore) | Explore in simulation |
| 6 | [Navigate on a saved map](#6-navigate-on-a-saved-map) | launch files on the robot |
| 7 | [Patrol](#7-patrol) | simulation |
| 8 | [Docking and auto-dock](#8-docking-and-auto-dock) | simulation |
| 9 | [Safety features](#9-safety-features) (lock, collision monitor, battery) | simulation |
| 10 | [Start everything at boot](#10-start-at-boot) | |
| 11 | [Calibrate](#11-calibrate) | |
| 12 | [Troubleshooting](#12-troubleshooting) | |

"Simulation" means a fake ESP32 whose wheels follow the commands, with real Nav2, not the
real robot yet. Lift the wheels off the ground the first time you try each feature.

---

## 0. One-time setup

### 0.1 Packages

```bash
sudo apt update && sudo apt upgrade        # includes ros-humble-tf2-ros 0.25.24: needed, see 12
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-cartographer-ros ros-humble-robot-localization ros-humble-rmw-cyclonedds-cpp \
  ros-humble-teleop-twist-keyboard python3-colcon-common-extensions \
  ros-humble-rosbridge-suite \
  ros-humble-twist-mux
```

| Package | Needed for | Without it |
|---|---|---|
| `ros-humble-rosbridge-suite` | the web page, the Android app, the project page's drive panel | `web_nav.launch.py` stops at start |
| `ros-humble-twist-mux` | manual-over-Nav2 priority and the Hold still lock | bringup warns and falls back to "last command wins" |
| `sudo apt upgrade` (tf2_ros 0.25.24) | a Nav2 costmap that keeps updating | Nav2 can freeze and drive on a stale costmap |

### 0.2 Workspace

```bash
git clone https://github.com/quangvu16/hoverboard-robot.git ~/robot_ws
cd ~/robot_ws && colcon build
```

Add to `~/.bashrc`. Source `local_setup.bash`, not `setup.bash`.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

After editing any YAML or launch file, rebuild that package (`colcon build --packages-select <pkg>`):
config is read from `install/`, not `src/`.

### 0.3 Firewall (only if ufw is on)

```bash
sudo ufw allow 9900:9902/udp   # ESP32 <-> PC: LiDAR, telemetry, wheel commands
sudo ufw allow 9090/tcp        # rosbridge websocket (web page, app, project page)
sudo ufw allow 8000/tcp        # the web page
sudo ufw allow 38232/tcp       # firmware updates over Wi-Fi (the board connects back)
```

### 0.4 Wi-Fi and the PC's address in the firmware

The PC and the ESP32 must be on the same Wi-Fi. In the sketch folder you use:

```bash
S=~/robot_ws/firmware/ldrobot_ld14p_esp32_mpu9250_ros        # ESP32-WROOM (the robot's board)
# S=~/robot_ws/firmware/ldrobot_ld14p_esp32s3_mpu9250_ros    # ESP32-S3-DevKitC-1 (see firmware/PINOUT.md)
cp $S/secrets.h.example $S/secrets.h
nano $S/secrets.h      # WIFI_SSID, WIFI_PASS, OTA_PASSWORD (choose a long one)
nano $S/config.h       # DEST_IP = this PC's address (ip -4 addr)
```

`secrets.h` is ignored by git. Never commit it.

---

## 1. Flash the ESP32

### 1.1 First time: USB

Plug the board in (WROOM: the USB port; S3: the port labelled **UART**). Then:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 $S
arduino-cli upload  --fqbn esp32:esp32:esp32:UploadSpeed=115200 -p /dev/ttyUSB0 $S
# ESP32-S3: --fqbn esp32:esp32:esp32s3 (and esp32:esp32:esp32s3:UploadSpeed=115200)
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200    # watch it boot
```

At boot the Serial Monitor prints, among other things:

- every pin it uses;
- `WiFi up as <esp32-ip>`;
- `web: drive page on http://<esp32-ip>/ and http://hoverboard.local/`;
- `ota: ready ...`, or `ota: OFF` if `secrets.h` has no password;
- the battery thresholds.

Upload at 115200 baud: faster uploads fail on this board. If an upload doesn't start, hold
**Boot**, tap **Reset**, then release **Boot**.

### 1.2 Every later update: over Wi-Fi

The robot must be **standing still**: the board ignores update requests while the wheels are
commanded or turning.

```bash
~/robot_ws/tools/ota_upload.sh                                          # WROOM sketch -> hoverboard.local
~/robot_ws/tools/ota_upload.sh ~/robot_ws/firmware/ldrobot_ld14p_esp32s3_mpu9250_ros <esp32-ip>
```

The script builds the sketch, finds the board and uploads, with the password from
`secrets.h`. The wheels are held at zero until the board reboots. A bad image can't brick
it: the old firmware stays in the other slot.

### 1.3 Firmware settings worth knowing

All in the sketch's `config.h`:

| Setting | Default | What |
|---|---|---|
| `DEST_IP` | `192.168.1.12` | the PC that receives LiDAR and telemetry |
| `WEB_DRIVE_ENABLED`, `WEB_DRIVE_MAX_RPM` | 1, 60 | the phone drive page and its speed cap |
| `BAT_LOW_V_PER_CELL`, `BAT_CRIT_V_PER_CELL`, `BAT_RECOVER_V_PER_CELL` | 3.55, 3.45, 3.70 | battery protection (×10 cells) |
| `OTA_ENABLED` | 1 | Wi-Fi updates (also needs `OTA_PASSWORD`) |
| `SONAR_STOP_MM`, `SONAR_SIDE_STOP_MM` | 350, 50 | the ESP32's own obstacle reflex |

---

## 2. Bring the robot up

**Run one stack at a time.** Two bringups fight over UDP 9902, and Cartographer and
slam_toolbox must never run together.

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

This starts:

- the hoverboard bridge (`/odom`, `/imu/data`, sonars, battery);
- the LiDAR receiver (`/scan`);
- the URDF and TF;
- the EKF (`/odom/filtered`);
- the safety chain: collision monitor → `twist_mux` → bridge.

Useful arguments:

| Argument | Default | Use |
|---|---|---|
| `ekf:=false` | `true` | raw wheel odometry only (Nav2 then gets no `/odom/filtered`) |
| `wheel_odom:=false` | `true` | LiDAR + IMU only; pass it to `cartographer_nav` too |
| `collision_monitor:=false` | `true` | Nav2's commands go straight to `twist_mux` |
| `cmd_mux:=false` / `true` | `auto` | never / always use `twist_mux` (auto = if installed) |
| `lidar:=none` | `udp` | bring up without the LiDAR |

At startup bringup prints the velocity chain it built, e.g.
`velocity chain: /cmd_vel -> collision_monitor -> /cmd_vel_safe -> twist_mux (+ /cmd_vel_manual, /cmd_vel_lock) -> /cmd_vel_mux -> hoverboard_udp_bridge`.

---

## 3. Check every link

```bash
ros2 topic echo /hoverboard/link_status     # esp:up mainboard:up fb=18B batt=38.9V batt_state=ok ...
ros2 topic hz /scan                         # ~6 Hz
ros2 topic hz /odom                         # ~50 Hz
ros2 topic echo /battery_state --once       # voltage, percentage (rough), health
ros2 topic echo /hoverboard/battery_state   # ok / low / critical  (latched)
ros2 topic echo /range/front --once         # sonar as sensor_msgs/Range; 2.0 m = no echo
ros2 topic echo /imu/calib_status           # "gyro 3" once the robot stood still ~2 s after power-on
ros2 topic echo /web/lidar --once           # (with web_nav running) LiDAR rate, points, nearest
```

| You see | Meaning |
|---|---|
| `waiting: no HBT1` | The ESP32 isn't reaching the PC. Check Wi-Fi, `DEST_IP` and the firewall. |
| `mainboard:down` | The ESP32 runs, but the mainboard UART doesn't. Check TX/RX, ground, and `VARIANT_USART`. |
| `batt_state=low` / `critical` | The ESP32 is halving the speed / holding the wheels. Charge the pack. |

---

## 4. Drive by hand

Pick one. All of them are stopped by the robot's own safety layers, and a manual command
overrides Nav2 while you hold it.

### 4.1 From the PC

```bash
ros2 run rqt_robot_steering rqt_robot_steering           # publishes /cmd_vel_manual (beats Nav2)
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # publishes /cmd_vel (through the collision monitor)
ros2 launch hoverboard_ctrl joy_launch.py                # gamepad
```

### 4.2 Web page (any browser on the Wi-Fi)

```bash
ros2 launch robot_web_nav web_nav.launch.py stack:=none      # only driving: no map, no Nav2
```

Open `http://<pc-ip>:8000`.

- **Drive:** drag the blue knob, or click the page and use `W` `A` `S` `D` / arrows.
  `Space` stops. Letting go stops.
- **Speed:** **Max speed** and **Max turn** cap the stick.
- **Status:** the **battery** label (volts, %, orange at low, red at critical), the **Sonar
  F · L · R** distances, and under Robot the **LiDAR** rate, points and nearest obstacle.
  With a map, the scan is drawn as red dots.
- **Hold still / Release:** the lock. It holds the robot against Nav2 and every stick until
  Release.

### 4.3 Android app

1. Install `~/RobotNav.apk`: `~/Android/Sdk/platform-tools/adb install -r ~/RobotNav.apk`, or
   copy it to the phone and open it.
2. Tap **Robot address** → `ws://<pc-ip>:9090`. The PC must be running `web_nav.launch.py`.

The Drive section has the stick, speed sliders, Stop, sonar distances, the battery label and
Hold still / Release.

Rebuild after changes:

```bash
cd ~/robot_ws/android_app && JAVA_HOME=~/Android/jdk ANDROID_HOME=~/Android/Sdk ./gradlew --no-daemon assembleRelease
cp app/build/outputs/apk/release/app-release.apk ~/RobotNav.apk
```

### 4.4 The project page's drive panel

Open `website/index.html` from a file or plain `http://`. Browsers block `ws://` from `https://`
pages. In chapter 09:

1. Type `<pc-ip>`.
2. Press **Connect**.
3. Hold the stick.

It also has Hold still / Release, Dock / Undock, battery and sonar.

### 4.5 Phone straight to the ESP32 (no PC, no ROS)

Open `http://hoverboard.local/` or `http://<esp32-ip>/` on a phone on the same Wi-Fi.

- **Controls:** the stick, two RPM sliders (25 rpm ≈ 0.25 m/s, capped at 60 rpm), and Stop.
- **Status:** live battery, sonars, and who has the wheels (web / ROS / idle).
- **Priority:** while you hold the stick it overrides ROS. ROS gets the wheels back 0.5 s
  after you let go.
- **Safety:** closing the page or losing Wi-Fi stops the robot.

---

## 5. Map a room

Start bringup (step 2) first.

### 5.1 By hand, with Cartographer (recommended)

```bash
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer        # mapping + Nav2 + web page
# or without the web page:  ros2 launch robot_cartographer cartographer_nav.launch.py
```

1. Drive slowly around the room with any stick from step 4, or click goals on the web page's
   map.
2. Save the map:
   ```bash
   ros2 run robot_cartographer save_map ~/m_map        # -> ~/m_map.pbstream + ~/m_map.yaml/.pgm
   ```

### 5.2 By itself: **Explore**

1. Launch in mapping mode (Cartographer with **no** `map:=`), as in 5.1. `explore_lite` starts
   with it, **paused**.
2. On the web page, press **Explore** (Navigation section). The robot drives to the edges of
   the known map until nothing unknown is left, then drives back to where it started.
3. When the page says **explore: finished, save the map**, save it as in 5.1.
- **Stop exploring** pauses it. A goal you click, **Cancel goal**, a patrol or docking also
  pause it, so they never fight over Nav2.
- From a shell: `ros2 topic pub -1 /web/explore std_msgs/String "data: start"` (or `stop`).
- `explore:=false` on `web_nav.launch.py` leaves it out; `explore:=true` forces it.
- The Explore button is on the `robot_web_nav` page. The Android app and the project page
  don't have it yet.

### 5.3 slam_toolbox instead

```bash
ros2 launch robot_bringup robot_slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/m_map
```

### 5.4 LiDAR + IMU only (encoders not used)

```bash
ros2 launch robot_bringup robot_bringup.launch.py wheel_odom:=false
ros2 launch robot_cartographer cartographer_nav.launch.py wheel_odom:=false
```

---

## 6. Navigate on a saved map

```bash
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream
# or Nav2 + AMCL:  ros2 launch robot_web_nav web_nav.launch.py stack:=amcl map:=$HOME/m_map.yaml
```

On `http://<pc-ip>:8000`, or in the app:

1. **Set pose.** Click where the robot really is and drag the way it faces. Do this first, so
   localization knows where it is.
2. **Goal.** Click where it should go. Drag to set the final heading. A plain click makes it
   face the way it drove.
3. **Watch it drive:** the status shows Driving / Arrived / Failed, distance left, ETA and
   recoveries. The planned path is blue.
4. **Cancel goal** stops it.
5. **Saved spots:** drive or navigate somewhere, type a name, press **Save here**. **Go**
   sends the robot back there. Spots are stored in that browser (or that phone).

RViz instead of the page: `ros2 launch robot_cartographer cartographer_nav.launch.py map:=$HOME/m_map.pbstream`
(its **2D Pose Estimate** and **Nav2 Goal** tools work).

---

## 7. Patrol

On a saved map (step 6), on the web page:

1. Save at least one spot (**Save here**). The spots are numbered on the map in patrol order.
2. Under Saved spots, set **Loops** (0 = until stopped) and **Wait s** (seconds at each spot).
3. Press **Patrol**. The next spot's number is filled in, and the status says e.g.
   `Patrol: to door (2/3, loop 1/3)` or `waiting 4 s`.
4. **Stop patrol**, **Cancel goal** or clicking a new goal ends it. So do docking and
   battery-low auto-dock.

The patrol runs in `web_nav_bridge` on the PC, so it keeps going with the page closed. An
unreachable spot gets one retry, then it's skipped.

From a shell:

```bash
ros2 topic pub -1 /web/patrol std_msgs/String \
  "data: '{\"spots\": [{\"name\": \"A\", \"x\": 1.0, \"y\": 0.0, \"yaw\": 0.0}, {\"name\": \"B\", \"x\": 1.0, \"y\": 1.0, \"yaw\": 1.57}], \"loops\": 2, \"wait_s\": 5}'"
ros2 topic pub -1 /web/patrol std_msgs/String "data: stop"
```

Patrol is on the `robot_web_nav` page. The Android app and the project page don't have it yet.

---

## 8. Docking and auto-dock

On a saved map (step 6). `web_nav.launch.py` starts the dock manager (`docking:=true`).

1. **Set the dock once.** Put the robot in its dock, localized (Set pose if needed), facing
   **out** of the dock. Press **Set dock here** and confirm. It's saved in
   `~/.ros/robot_dock.json` and survives restarts. The dock is drawn on the map as a dashed
   square with its back wall.
2. **Dock.** Nav2 drives to a point 0.7 m in front of the dock, facing away. Then the robot
   **reverses** in and stops 1 cm short. The status shows Docking… → Docked.
3. **Undock.** It drives 0.7 m forwards, out of the dock.
4. **Auto-dock** (checkbox, on by default). When the ESP32 reports battery **low** (below
   35.5 V, speed already halved), the robot docks by itself and ends any goal or patrol.
5. **Cancel** stops docking or undocking.

| Where | Controls |
|---|---|
| Web page / Android app, Dock section | Set dock here, Dock, Undock, Cancel, auto-dock checkbox |
| Project page drive panel | Dock / Undock (Cancel while busy) |
| Shell | `ros2 topic pub -1 /dock/command std_msgs/String "data: dock"` (also `save`, `undock`, `cancel`, `auto on`, `auto off`); `ros2 topic echo /dock/status` |

Limits:

- With no camera, the dock is only as precise as the localization.
- Charging needs contacts on the dock wired to the hoverboard's charge port.
- The robot can't sense charging current.

Without the web page: `ros2 launch robot_docking docking.launch.py` (Nav2 must be running).

---

## 9. Safety features

They are always on. This is how to use and check them.

| Feature | How to use / check |
|---|---|
| **Hold still lock** | **Hold still** / **Release** on the web page, app or project panel, or `ros2 topic pub -1 /cmd_vel_lock std_msgs/Bool "data: true"` (`false` releases). Blocks Nav2 and every stick. Needs `twist_mux`. |
| **Manual beats Nav2** | Grab any stick during a goal: the goal is cancelled and you drive. Nav2 gets the wheels back 0.5 s after you let go. |
| **Collision monitor** | Nav2 slows to 40 % with something within 0.60 m ahead, and stops at 0.32 m (LiDAR and sonars). RViz: add `/polygon_slowdown` and `/polygon_stop`. Zones in `src/robot_bringup/param/collision_monitor_hoverboard.yaml`. |
| **Sonars in Nav2** | The local costmap's range layer marks glass, chair legs and low objects from about 0.5 m. Topics `/range/{front,left,right}`. |
| **ESP32 sonar reflex** | Works with Wi-Fi down: reverses below 350 mm in front, turns away below 50 mm on a side. The `obstacle ahead` label shows it. |
| **Battery protection** | ESP32: low (35.5 V) halves the speed and beeps every 30 s; critical (34.5 V) holds the wheels and beeps twice every 5 s. Normal again only above 37.0 V (charged). `/hoverboard/battery_state`, `/battery_state`. |
| **Timeouts** | Bridge 0.5 s without commands, ESP32 1 s without HBC1, mainboard 0.8 s without UART: each stops the wheels. |

---

## 10. Start at boot

```bash
sudo cp ~/robot_ws/tools/robot-bringup.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now robot-bringup
journalctl -u robot-bringup -f
```

- The unit runs `tools/robot_bringup.sh` as user `vu`. Edit `User=` and `ExecStart=` for
  another machine.
- Extra bringup arguments go in `ROBOT_ARGS` in the unit, e.g. `Environment=ROBOT_ARGS=ekf:=false`.
- Then start the web page / Nav2 with `web_nav.launch.py` as in steps 5 and 6. Don't start a
  second bringup.

---

## 11. Calibrate

Do this once the LiDAR and IMU are fixed to the chassis:

```bash
python3 ~/robot_ws/tools/lidar_mirror_test.py        # is /scan mirrored or rotated, and does it turn with the robot?
python3 ~/robot_ws/tools/calib_wheel_separation.py   # odom yaw vs Cartographer yaw -> wheel_separation_m
```

Then:

1. Set the real sensor poses in `src/hoverboard_description/urdf/hoverboard.urdf` (LiDAR, IMU,
   sonars).
2. Set `wheel_separation_m` in `src/hoverboard_base_node/config/hoverboard_base.yaml`.
3. Check the collision monitor zones.
4. Compare `/hoverboard/battery_volts` with a multimeter. If they differ, fix `BAT_CALIB_*` in
   the mainboard firmware: the battery thresholds trust that reading.

Rebuild the changed packages.

---

## 12. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Obstacles in RViz's local costmap stop updating; Nav2 drives on, `clear_entirely_local_costmap` times out | tf2_ros 0.25.23 deadlock | `sudo apt upgrade`, restart Nav2 |
| Web page says **no rosbridge** | `web_nav.launch.py` not running, rosbridge not installed, or port 9090 blocked | step 0.1 / 0.3 |
| Robot won't move at all | Lock on, battery critical, or a second bridge running | `data: false` on `/cmd_vel_lock`; check `/hoverboard/battery_state`; `ps aux \| grep bridge` |
| Only half speed | Battery low | Charge |
| Nav2 creeps or stops near walls | Collision monitor zones | Watch `/polygon_slowdown`, `/polygon_stop`; tune the YAML |
| Explore button greyed (**not running**) | Not in mapping mode | `stack:=cartographer` with no `map:=` |
| Explore **finished** at once | No unknown space reachable | Drive somewhere new, or the map is complete: save it |
| Dock: *Nav2 is not running* / *could not reach the staging pose* | No nav stack, or not localized | Start with a map and Set pose first |
| Docks beside the charger | Localization off | Re-localize, or Set dock here again |
| `ota_upload.sh`: *No response from the ESP* | Robot moving, no `OTA_PASSWORD`, or ufw | Stop the robot; step 0.4 + USB flash; `sudo ufw allow 38232/tcp` |
| `hoverboard.local` doesn't resolve | No mDNS on that device | Use `<esp32-ip>` |
| Nav2: `frame "odom" does not exist` | Bringup and Cartographer in different `wheel_odom` modes | Same argument for both |
| Edited YAML has no effect | Config is read from `install/` | `colcon build --packages-select <pkg>` |
| `/imu/data` missing | IMU wiring, or gyro not calibrated yet | Keep still 2 s after power-on; check SDA/SCL/3V3 |
