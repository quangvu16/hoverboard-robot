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
| `/cmd_vel_manual` | geometry_msgs/Twist | in | manual driving: `linear.x` m/s, `angular.z` rad/s. Resend at least every 0.5 s |
| `/ultrasonic/obstacle_near` | std_msgs/Bool | out | front sonar is blocking |
| `/web/robot_pose` | geometry_msgs/PoseStamped | out | 5 Hz |
| `/web/nav_status` | std_msgs/String (JSON) | out | state, goal, distance_remaining, eta_s, nav_time_s, recoveries, stack, server_ready |
| `/map`, `/plan` | OccupancyGrid, Path | out | straight from the stack |

`web/roslib.min.js` is roslib 1.4.1 (BSD), copied in so the page works without internet.
