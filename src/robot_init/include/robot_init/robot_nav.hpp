// nav_adapter.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nlohmann/json.hpp>

class ROBOT;
class Bus;

class NavAdapter {
public:
  NavAdapter(ROBOT* robot, Bus* bus);
  void publish_goal(double x, double y, double yaw_rad);
  void follow_waypoints_json(const std::vector<nlohmann::json>& wps, bool release_mode);
  void rotate_to_yaw(double target_yaw_rad);

private:
  ROBOT* robot_;
  Bus*   bus_;

  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav2_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
