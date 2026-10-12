#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class ROBOT;

class Bus {
public:
  explicit Bus(ROBOT* robot);

  void state(const std::string& s);
  void move_type(const std::string& s);
  void action(const std::string& s);
  void charge(bool on);
  void error(bool er);
  void release_scan(bool on);
  void cancel_task(int16_t t);
  void goal(const geometry_msgs::msg::PoseStamped& p);
  void path(const nav_msgs::msg::Path& p);
  void path_release(const nav_msgs::msg::Path& p);
  void nodeID(const std::string& s);
  void rcsState(bool on);

private:
  ROBOT* robot_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_type_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charge_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr release_scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cancel_task_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_release_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nodeID_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rcs_state_pub_; 
};
