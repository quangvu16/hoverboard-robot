// esp_tcp_server/ros_bridge.hpp
#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

namespace esp_tcp_server {

class RosBridge
{
public:
  using SendMoveCb = std::function<void(float, float)>;

  explicit RosBridge(rclcpp::Node* node);

  // Called by EspNode when a DeviceState json arrives (raw json string)
  void pub_state_json(const std::string& s);

  void pub_link(const std::string& s);

  void set_send_move_cb(SendMoveCb cb) { send_move_cb_ = std::move(cb); }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_manual_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);


  rclcpp::Node* node_{nullptr};

  // Raw debug pubs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_link_;

  // cmd_vel sub
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_manual_cmd_vel_;

  // Odometry pub
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  //hcsr04 pub
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ultrasonic_distance_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ultrasonic_obstacle_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Frames (change here if your system uses base_footprint, etc.)
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_footprint";   // or "base_footprint"

  SendMoveCb send_move_cb_;

  bool ultrasonic_valid_ = false;
  bool obstacle_near_ = false;
  int ultrasonic_distance_mm_ = 0;
  int ultrasonic_stop_mm_ = 60;
  int ultrasonic_near_count_ = 0;
  int ultrasonic_far_count_ = 0;
};
} // namespace esp_tcp_server
