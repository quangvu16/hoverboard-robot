#include "robot_init/robot_publish.hpp"
#include "robot_init/robot.hpp"

Bus::Bus(ROBOT* robot) : robot_(robot) {
  state_pub_        = robot_->create_publisher<std_msgs::msg::String>("/robot/state", 10);
  // Bus/NavAdapter
  move_type_pub_ = robot_->create_publisher<std_msgs::msg::String>(
    "/task/move_type", rclcpp::QoS(1).transient_local().reliable());
  action_pub_       = robot_->create_publisher<std_msgs::msg::String>("/robot_action", 10);
  charge_pub_       = robot_->create_publisher<std_msgs::msg::Bool>("/task/charge", 10);
  error_pub_        = robot_->create_publisher<std_msgs::msg::Bool>("/robot/error", 10);
  release_scan_pub_ = robot_->create_publisher<std_msgs::msg::Bool>("/task/release_scan", 10);
  cancel_task_pub_  = robot_->create_publisher<std_msgs::msg::Int16>("/task/cancel", 10);
  goal_pub_         = robot_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  path_pub_         = robot_->create_publisher<nav_msgs::msg::Path>("/path", 10);
  path_release_pub_ = robot_->create_publisher<nav_msgs::msg::Path>("/path_release", 10);
  nodeID_pub_       = robot_->create_publisher<std_msgs::msg::String>("/robot/node_id", 10);
  rcs_state_pub_    = robot_->create_publisher<std_msgs::msg::Bool>("/robot/rcs_status", 10);
}

void Bus::state(const std::string& s){
  std_msgs::msg::String m; m.data = s;
  state_pub_->publish(m);
}

void Bus::move_type(const std::string& s){
  std_msgs::msg::String m; m.data = s;
  move_type_pub_->publish(m);
}

void Bus::action(const std::string& s){
  std_msgs::msg::String m; m.data = s;
  action_pub_->publish(m);
}

void Bus::charge(bool on){
  std_msgs::msg::Bool m; m.data = on;
  charge_pub_->publish(m);
}

void Bus::error(bool er){
  std_msgs::msg::Bool m; m.data = er;
  error_pub_->publish(m);
}

void Bus::release_scan(bool on){
  std_msgs::msg::Bool m; m.data = on;
  release_scan_pub_->publish(m);
}

void Bus::cancel_task(int16_t t){
  std_msgs::msg::Int16 m; m.data = t;
  cancel_task_pub_->publish(m);
}

void Bus::goal(const geometry_msgs::msg::PoseStamped& p){
  goal_pub_->publish(p);
}

void Bus::path(const nav_msgs::msg::Path& p){
  path_pub_->publish(p);
}

void Bus::path_release(const nav_msgs::msg::Path& p){
  path_release_pub_->publish(p);
}

void Bus::nodeID(const std::string& s) {
  std_msgs::msg::String m; m.data = s;
  nodeID_pub_->publish(m);
}

void Bus::rcsState(bool on){
  std_msgs::msg::Bool m; m.data = on;
  rcs_state_pub_->publish(m);
}