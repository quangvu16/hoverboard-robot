-- Cartographer 2D for the hoverboard robot from LiDAR + IMU only, no wheel
-- encoders: cartographer_nav.launch.py wheel_odom:=false, with
-- robot_bringup.launch.py wheel_odom:=false (no EKF, bridge TF off).
--
-- Frames:
--   map -> odom -> base_footprint   published HERE: Cartographer's local
--                                   SLAM (scan matching + IMU) is the odometry
--   base_footprint -> base_link -> {base_scan, imu_link}   robot_state_publisher
--
-- The IMU (/imu/data, MPU9250 via hoverboard_udp_bridge_node) gives gravity
-- and the turn rate between scans; tracking_frame is its frame, as
-- Cartographer requires when IMU data is used. Its pose in the URDF's
-- imu_joint must therefore be right: robot still -> accel z ~ +9.8 in
-- base_link, turning left -> angular_velocity.z > 0.
-- Cartographer waits for the first IMU message before it uses any scan, and
-- the bridge holds /imu/data until the gyro bias is learned (robot still ~2 s).

include "robot_2d.lua"

options.tracking_frame = "imu_link"
options.published_frame = "base_footprint"
options.odom_frame = "odom"
options.provide_odom_frame = true     -- nothing else owns odom -> base_footprint
options.use_odometry = false          -- no wheel encoders

TRAJECTORY_BUILDER_2D.use_imu_data = true

-- Without wheel odometry the scan matcher alone finds translation; the
-- search window and weights from robot_2d.lua (sized for poor odometry)
-- already allow that. Rotation comes mostly from the gyro, so it may be
-- trusted a little more than in robot_2d.lua.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.

return options
