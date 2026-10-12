-- Pure localization on a saved map from LiDAR + IMU only (no wheel encoders):
-- robot_2d_imu.lua plus the same localization settings as
-- robot_2d_localization.lua (keep the two in step).
--   ros2 launch robot_cartographer cartographer_nav.launch.py wheel_odom:=false map:=$HOME/m_map.pbstream

include "robot_2d_imu.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

return options
