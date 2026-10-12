# full_coverage_path_planning

Full coverage path planning ("cover every free cell of the map", as for a
cleaning or inspection robot) for the hoverboard robot. A ROS 2 Humble / Nav2
port of
[globaledgesoft/Full-Coverage-Path-Planning-on-TurtleBot3-Qualcomm-RB5](https://github.com/globaledgesoft/Full-Coverage-Path-Planning-on-TurtleBot3-Qualcomm-RB5),
which is ROS 1 Melodic (catkin, move_base_flex, tracking_pid).

| Original (ROS 1) | Here (ROS 2) |
|---|---|
| nobleo `full_coverage_path_planner` SpiralSTC in move_base_flex | same algorithm, nobleo's own `ros2` branch as a Nav2 planner plugin — vendored in `../full_coverage_path_planner` (upstream commit in `UPSTREAM_COMMIT`; only change: `backward_ros` made optional) |
| `tracking_pid` + `path_interpolator` (ROS 1 only) | Nav2 controller server, Regulated Pure Pursuit (`FollowCoverage`) |
| `intial_pose_estimation.py`: spin 20 s at 2 rad/s, then a dummy goal | `coverage_runner`: one gentle localization turn, then plans ONCE and follows |
| `coverage_progress` over a hand-set rectangle (`coverage_area_offset`, 19.25 m) | `coverage_progress` over the map's free cells, no setup |
| TurtleBot3 burger, `robot_footprint_size` 0.2 | hoverboard, 0.33 m square: robot_radius 0.17, tool_radius 0.165 |

Not ported: the repo's alternative `robot_cleaning_services/cleaning.py`
(waypoint grid via shapely + move_base) and its custom `robot_global_planner`
move_base plugin — Spiral-STC above is the path its main launch
(`rb5_FCPP.launch`) actually uses. Gazebo worlds and TurtleBot3 maps are not
needed on the real robot.

## Run

```bash
ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard
ros2 launch full_coverage_path_planning coverage.launch.py map:=$HOME/maps/home.yaml
```

1. In RViz, "2D Pose Estimate" where the robot is.
2. After `start_delay` (15 s) the robot turns once to settle AMCL, plans the
   coverage path (blue in RViz) and drives it. The coverage map fills in and
   the log prints the percentage.

```bash
ros2 topic echo /coverage_progress                               # 0..1
ros2 service call /coverage_runner/cancel std_srvs/srv/Trigger   # stop
ros2 service call /coverage_runner/start std_srvs/srv/Trigger    # plan again from here
ros2 service call /coverage_progress/reset std_srvs/srv/Trigger
```

Launch arguments: `map`, `tool_radius` (0.165; smaller = more overlap),
`auto_start`, `start_delay`, `localize_spin_deg` (0 = no turn), `rviz`,
`params_file` (Nav2 base parameters, default hoverboard.yaml).

Needs the map to be built first (Cartographer / slam_toolbox, then
`map_saver_cli`) and the LiDAR mounted on the chassis — see
`~/robot_ws/tools/lidar_mirror_test.py`.
