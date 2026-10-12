#!/usr/bin/env bash
# Started by robot-bringup.service. Same environment as ~/.bashrc:
# local_setup.bash, never setup.bash.
set -e
source /opt/ros/humble/setup.bash
source "$HOME/slam_toolbox_ws/install/local_setup.bash"
source "$HOME/robot_ws/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
# Extra launch arguments come from the unit's ROBOT_ARGS (e.g. "ekf:=false").
exec ros2 launch robot_bringup robot_bringup.launch.py $ROBOT_ARGS
