# robot_docking

Auto-docking for the hoverboard robot. You save a dock on the map; the robot backs into it
on request, and by itself when the battery runs low.

```bash
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream   # includes it (docking:=true)
ros2 launch robot_docking docking.launch.py                                                 # or alone, with Nav2 running
```

No extra packages: it uses Nav2's `navigate_to_pose` and TF.

## Using it

1. Park the robot in the dock (by hand or with the drive stick), localized on the map.
2. Press **Set dock here** on the web page, or run
   `ros2 topic pub -1 /dock/command std_msgs/String "data: save"`.
   The pose is stored in `~/.ros/robot_dock.json` and survives restarts.
3. **Dock** / **Undock** / **Cancel**, or `data: dock` / `data: undock` / `data: cancel`.
4. **Auto-dock** (on by default; the page checkbox or `data: auto off`). When the ESP32
   reports `low` (below 35.5 V, speed already halved), the robot docks by itself and replaces
   any Nav2 goal. It doesn't wait for `critical`, because the ESP32 holds the wheels at zero
   then.

## How it docks

```
            dock (saved robot pose)          staging pose
   wall ▐ [robot] ──────── 0.7 m ──────────► ●
          ◄── 2. reverse in (dock_manager)   1. Nav2 drives here, facing away
```

- **1. Nav2 to the staging pose.** Nav2 drives forwards to a point 0.7 m in front of the
  docked robot, facing away from the dock.
- **2. Reverse in.** `dock_manager` backs the robot in at up to 0.10 m/s. It steers onto the
  dock axis from the TF pose and stops 1 cm short of the saved pose. If that takes over 40 s,
  it goes back to the staging pose and tries once more.
- **Undock.** It drives forwards 0.7 m, out to the staging pose.
- **Why backwards.** The front HC-SR04 backs the robot away from anything nearer than
  350 mm, and the collision monitor only watches the front. Backing in keeps both pointed
  away from the dock.
- **Same safety chain.** The moves go through the same chain as Nav2's: `/cmd_vel` →
  collision monitor → `twist_mux`. A stick or `/cmd_vel_lock` still wins.

Limits:

- **No camera.** The dock is where the map says it is, so the final position is only as good
  as the localization. Simulation got within 3 cm of the axis and 4° of heading.
- **No charge sensing.** The mainboard measures motor current, not battery current, so the
  software can't tell whether it's charging. Charging also needs contacts on the dock wired
  to the hoverboard's charge port.
- **Not Nav2's docking server.** It was tried first. Its Humble release
  (`ros-humble-opennav-docking` 0.0.2) got backwards docking wrong in simulation: with one
  dock convention the robot turned round at the staging pose, with the other it undocked by
  reversing into the wall. With a map-pose dock and no camera it would add nothing. If an
  AprilTag dock detector is added later, revisit it.

Parameters (`dock_manager`): `staging_distance` 0.7, `approach_speed` 0.10, `min_speed`
0.04, `k_angular` 1.5, `max_angular` 0.6, `dock_tolerance` 0.01, `move_timeout` 40,
`nav_timeout` 180, `retries` 1, `auto_dock`, `dock_file`.

## Topics

| Topic | Type | | |
|---|---|---|---|
| `/dock/command` | `std_msgs/String` | in | `save`, `dock`, `undock`, `cancel`, `auto on`, `auto off` |
| `/dock/status` | `std_msgs/String` (JSON) | out, latched | `state` (no_dock, idle, docking, docked, undocking, failed), `message`, `dock {x,y,yaw}`, `auto`, `battery`, `percentage` |
| `/dock/pose` | `geometry_msgs/PoseStamped` | out, latched | the saved dock: the robot's pose when docked, map frame |
| `/cmd_vel` | `geometry_msgs/Twist` | out | only while reversing in or driving out |
| `/battery_state` | `sensor_msgs/BatteryState` | in | from `hoverboard_udp_bridge`; percentage for the status |
| `/hoverboard/battery_state` | `std_msgs/String` | in | ESP32 `ok` / `low` / `critical`; `low` triggers auto-dock |
