# robot_web_nav

Drive the robot from a web page over a websocket: by hand with a joystick, or by sending
Nav2 goals on the map. Open the page in any browser on
the same network as the robot PC: phone, tablet or laptop, with nothing to install on it.

```
browser (index.html + roslib.js)  <--websocket :9090-->  rosbridge_server
                                                              |
         /web/goal, /web/cancel, /initialpose  ------->  web_nav_bridge --> Nav2 NavigateToPose
         /map, /plan, /web/robot_pose, /web/nav_status <--     (TF map -> base_footprint)
         /cmd_vel_manual (joystick)  -------------------->  hoverboard_udp_bridge --> wheels
```

## Install once

```bash
sudo apt install ros-humble-rosbridge-suite
cd ~/robot_ws && colcon build --packages-select robot_web_nav
source install/local_setup.bash
sudo ufw allow 9090/tcp && sudo ufw allow 8000/tcp     # only if ufw is enabled
```

## Run

Bringup must be running (`robot_bringup.launch.py` or `robot-bringup.service`).
Then choose the navigation stack:

```bash
# Cartographer: localization on a saved map (or no map:= = map while navigating)
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream

# Nav2 + AMCL on a map_saver / slam_toolbox map
ros2 launch robot_web_nav web_nav.launch.py stack:=amcl map:=$HOME/m_map.yaml

# Nav stack already running in another terminal: only the web parts
ros2 launch robot_web_nav web_nav.launch.py stack:=none
```

Open `http://<robot-pc-ip>:8000`. The page connects to `ws://<same host>:9090`.
To use another websocket address, add `?ws=ws://host:port` to the page URL.

`wheel_odom:=false` (cartographer only) must match the bringup, as it does for
`cartographer_nav.launch.py`. RViz is off for cartographer (`rviz:=true` turns it back
on). The amcl launch always opens RViz.

## Using the page

- **Drive** (joystick, top right): drag the blue knob to drive, and let go to stop. Up is
  forward, and right turns right. The keyboard works too: `W` `A` `S` `D` or the arrow
  keys, and `Space` stops. Two sliders cap the speed (default 0.2 m/s, up to 0.5) and the
  turn rate (default 0.8 rad/s, up to 2). The pad works without a map or Nav2: use
  `stack:=none` to only drive. Grabbing the stick during a Nav2 goal cancels the goal.
  Safety:
  - commands go out 10 times a second only while you hold the stick or a key
  - the bridge stops the wheels 0.5 s after the last command, so a closed tab or a
    dropped Wi-Fi link stops the robot
  - switching tabs or windows stops the robot at once
  - the ESP32's sonar reflex still overrides you; the `obstacle ahead` pill shows it
- **Hold still** / **Release** (under Stop): the `twist_mux` lock on `/cmd_vel_lock`. It holds
  the robot still against Nav2 and every stick, and greys the stick. The page re-sends it
  every 2 s while holding, and the label shows what the topic last carried, from any client.
  It needs `ros-humble-twist-mux` on the robot PC (robot_bringup `cmd_mux`).
- **Explore** (Navigation section): while mapping (`stack:=cartographer` with no `map:=`),
  `explore_lite` starts paused with the launch. Explore drives the robot to the edges of the
  known map until nothing is unknown, then returns to where it started. When it says
  *finished*, save the map (`ros2 run robot_cartographer save_map ~/m_map`). Stop exploring
  pauses it. A goal from the page, Cancel, a patrol or docking also pauses it, so they never
  fight over Nav2.
- **Patrol** (Saved spots): visits the saved spots in order, with **Loops** (0 = until stopped)
  and **Wait s** at each spot. It runs in `web_nav_bridge` on the robot PC, so it keeps going
  with the page closed. An unreachable spot gets one retry, then it's skipped. A goal from the
  page, Cancel, Stop patrol or docking ends it. The spots are numbered on the map, and the
  next one is filled.
- **Dock** section: **Set dock here** (confirms first; the robot must be in its dock),
  **Dock**, **Undock**, **Cancel**, and **Dock by itself when the battery is low**. The dock is
  drawn on the map as a dashed square with its back wall. It needs `robot_docking`, which this
  launch starts (`docking:=true`).
- **Battery** pill (top): the ESP32's voltage and state from `/hoverboard/battery_state` and
  `/hoverboard/battery_volts`. It turns orange at low (the ESP32 halves the speed) and red at
  critical (the wheels are held until the pack is charged).
- **Sonar F · L · R**: distances in metres from `/range/{front,left,right}`. `clear` = no echo
  within 2 m.
- **Goal**: click where the robot should go. Drag to set the final heading. A plain
  click makes the robot face the way it drove. A new goal replaces the active one.
- **Set pose**: click where the robot really is, and drag the way it faces. This publishes
  `/initialpose`. AMCL reads it directly. Cartographer reads it through `initialpose_relay`,
  which only runs when a `.pbstream` map is loaded.
- **Cancel goal** stops the active goal.
- **Saved spots** store named positions in this browser (localStorage), with a Go button for each.
- Status shows the state (driving / arrived / failed / cancelled), distance left,
  ETA, driving time and recoveries, all from Nav2's feedback.

## Topics (for your own clients)

Any websocket client that speaks the rosbridge v2 protocol works, for example roslibpy in Python.
To drive, send this JSON over `ws://<robot-pc-ip>:9090` every 100 ms, and send zeros to stop:

```json
{"op": "publish", "topic": "/cmd_vel_manual",
 "msg": {"linear": {"x": 0.2, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0.5}}}
```


| Topic | Type | Dir | |
|---|---|---|---|
| `/web/goal` | geometry_msgs/PoseStamped | in | frame `map` |
| `/web/cancel` | std_msgs/Empty | in | |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | in | |
| `/cmd_vel_lock` | std_msgs/Bool | in/out | twist_mux lock: true = hold still |
| `/web/explore` | std_msgs/String | in | `start` / `stop` (explore_lite on `/explore/resume`) |
| `/web/patrol` | std_msgs/String | in | JSON `{"spots": [{"name","x","y","yaw"}], "loops": n, "wait_s": s}` or `stop` |
| `/dock/command` | std_msgs/String | in | `save`, `dock`, `undock`, `cancel`, `auto on`, `auto off` |
| `/dock/status` | std_msgs/String (JSON) | out | state, message, dock pose, auto, battery, percentage; latched |
| `/battery_state` | sensor_msgs/BatteryState | out | voltage, rough percentage |
| `/hoverboard/battery_state` | std_msgs/String | out | `ok` / `low` / `critical` / `unknown`, latched |
| `/hoverboard/battery_volts` | std_msgs/Float32 | out | |
| `/range/{front,left,right}` | sensor_msgs/Range | out | HC-SR04s; `max_range` (2 m) = no echo |
| `/cmd_vel_manual` | geometry_msgs/Twist | in | manual driving: `linear.x` m/s, `angular.z` rad/s. Resend at least every 0.5 s |
| `/ultrasonic/obstacle_near` | std_msgs/Bool | out | front sonar is blocking |
| `/web/robot_pose` | geometry_msgs/PoseStamped | out | 5 Hz |
| `/web/nav_status` | std_msgs/String (JSON) | out | state, goal, distance_remaining, eta_s, nav_time_s, recoveries, stack, server_ready, explore (unavailable/paused/exploring/finished), patrol (index, count, name, loop, loops, waiting_s) |
| `/map`, `/plan` | OccupancyGrid, Path | out | straight from the stack |

`web/roslib.min.js` is roslib 1.4.1 (BSD), copied in so the page works without internet.
