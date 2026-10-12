-- Cartographer 2D pure localization for the hoverboard robot: same sensors and
-- tuning as robot_2d.lua, against a map saved with
--   ros2 run robot_cartographer save_map ~/m_map
-- and loaded frozen by
--   ros2 launch robot_cartographer cartographer_nav.launch.py map:=$HOME/m_map.pbstream
--
-- The saved map is trajectory 0 (frozen, never changed). The robot drives a new
-- trajectory that is matched against it; only the last few of its own submaps
-- are kept, so memory does not grow however long it runs. RViz's
-- "2D Pose Estimate" restarts that trajectory at the clicked pose
-- (initialpose_relay); without it, the global search below finds the robot.

include "robot_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Match against the frozen map more often than while mapping (35 nodes),
-- so the robot snaps onto it quickly after start and after a 2D Pose Estimate.
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

return options
