# robot_web_nav

Drive Nav2 from a web page over a websocket. Open the page in any browser on
the same network as the robot PC: phone, tablet or laptop, with nothing to install on it.

```
browser (index.html + roslib.js)  <--websocket :9090-->  rosbridge_server
                                                              |
         /web/goal, /web/cancel, /initialpose  ------->  web_nav_bridge --> Nav2 NavigateToPose
         /map, /plan, /web/robot_pose, /web/nav_status <--     (TF map -> base_footprint)
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

| Topic | Type | Dir | |
|---|---|---|---|
| `/web/goal` | geometry_msgs/PoseStamped | in | frame `map` |
| `/web/cancel` | std_msgs/Empty | in | |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | in | |
| `/web/robot_pose` | geometry_msgs/PoseStamped | out | 5 Hz |
| `/web/nav_status` | std_msgs/String (JSON) | out | state, goal, distance_remaining, eta_s, nav_time_s, recoveries, stack, server_ready |
| `/map`, `/plan` | OccupancyGrid, Path | out | straight from the stack |

`web/roslib.min.js` is roslib 1.4.1 (BSD), copied in so the page works without internet.
