## This copy: adapted for the hoverboard robot

Upstream: kaiaai/auto_mapper (commit in `UPSTREAM_COMMIT`). Changes:

* **Real robot:** `is_sim` defaults to false.
* **Nav2:** `nav2_bringup/navigation_launch.py` with `robot_navigation/param/hoverboard.yaml`
  (no map_server/AMCL — slam_toolbox owns `/map` and `map→odom`). The bundled TurtleBot-sized
  `nav2_params.yaml` and `bringup_launch.py` were removed.
* **slam_toolbox** (`config/mapper_params_online_async.yaml`): `mode: mapping` (was
  `localization`, which cannot build a new map), LD14 range 8 m, `min_laser_range` 0.30 m
  (robot body), faster map updates, smaller travel thresholds for a 0.25 m/s robot.
* **Explorer** (`src/auto_mapper.cpp`): frontiers whose Nav2 goal aborts are blacklisted
  (upstream re-sent the same unreachable frontier forever); fixed a dangling reference
  captured in the goal-response callback; thresholds are parameters
  (`min_distance_to_frontier`, `min_frontier_size`, `blacklist_radius`).
* **Resume** of a saved pose graph only with `resume:=true` (was automatic whenever a file
  existed, which mislocates the robot if it was moved).

```bash
ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard
ros2 launch auto_mapper auto_mapper.launch.py map_path:=$HOME/maps/flat rviz:=true
# when it logs "NO BOUNDARIES FOUND" / "Stopped", the map is in ~/maps/flat.yaml
ros2 launch robot_navigation navigation2.launch.py map:=$HOME/maps/flat.yaml
```

Needs the LiDAR mounted on the chassis (`~/robot_ws/tools/lidar_mirror_test.py`).

---

<h3 align="center">auto_mapper</h3>

  <p align="center">
    Package for auto exploring frontiers and generating a map with ROS 2 Humble / Iron
  </p>

<video src="https://github.com/Omar-Salem/auto_mapper/blob/master/Demo.mp4" width="320" height="240" controls></video>

### Prerequisites

* [ROS 2 Humble / Iron](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your Ubuntu 22.04
* [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [nav2](https://github.com/ros-planning/navigation2)
* [rviz2](https://github.com/ros2/rviz)

### Installation & usage

1. Clone the repo
```sh
git clone https://github.com/Omar-Salem/auto_mapper
```
2. Build the ROS 2  workspace
```sh
cd {workspace_dir}
```
```sh
colcon build --packages-select auto_mapper
```
3. Source the ROS Workspace
```sh
. install/setup.bash
```

4. Launch the mapper
```sh
ros2 launch auto_mapper auto_mapper.launch.py map_path:=~/map
```

5. Launch turtlebot3
```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```


### Gotchas
* If you receive `Failed to write file: Exception output stream error`, change the map file path to a dir where write access is granted.
* To use with a custom robot, or if the Twist messages aren't published to `/cmd_vel`, uncomment `SetRemap(src='/cmd_vel', dst='{/your/custom_cmd_vel}}')` in `auto_mapper.launch.py`

### Contact

Omar Salem - [LinkedIn](https://www.linkedin.com/in/omar-salem-4564a590/)



### Acknowledgements
* [m-explore](https://github.com/hrnr/m-explore)
