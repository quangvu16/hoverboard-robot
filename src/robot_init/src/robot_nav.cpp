// nav_adapter.cpp
#include "robot_init/robot_nav.hpp"
#include "robot_init/robot.hpp"
#include "robot_init/robot_publish.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>
#include <algorithm>

NavAdapter::NavAdapter(ROBOT* robot, Bus* bus)
: robot_(robot), bus_(bus)
{
  nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      robot_, "navigate_through_poses");

  cmd_vel_pub_ = robot_->create_publisher<geometry_msgs::msg::Twist>("/bcr_bot/cmd_vel", 10);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(robot_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void NavAdapter::publish_goal(double x, double y, double yaw_rad) {
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = robot_->now();
  p.header.frame_id = "map";
  tf2::Quaternion q; q.setRPY(0,0,yaw_rad);
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation = tf2::toMsg(q);
  bus_->goal(p);
}

void NavAdapter::follow_waypoints_json(const std::vector<nlohmann::json>& wps, bool release_mode) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = robot_->now();

  for (const auto& wp : wps) {
    if (!wp.contains("Position")) continue;
    double x = wp["Position"][0].get<float>() / 1000.0f;
    double y = wp["Position"][1].get<float>() / 1000.0f;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = x; ps.pose.position.y = y;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  if (path.poses.empty()) return;
  if (release_mode) bus_->path_release(path);
  else              bus_->path(path);
}

void NavAdapter::rotate_to_yaw(double target_yaw_rad) {
  rclcpp::Rate rate(10);
  const double tol  = 0.02;
  const double k_p  = 1.4;
  const double wmax = 0.3;

  while (rclcpp::ok() && robot_->is_onsite && !robot_->is_error) {
    geometry_msgs::msg::TransformStamped tfm;
    try {
      tfm = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (...) { rate.sleep(); continue; }

    tf2::Quaternion q(
      tfm.transform.rotation.x,
      tfm.transform.rotation.y,
      tfm.transform.rotation.z,
      tfm.transform.rotation.w
    );
    double r,p,yaw; tf2::Matrix3x3(q).getRPY(r,p,yaw);
    double err = angles::shortest_angular_distance(yaw, target_yaw_rad);
    if (std::abs(err) < tol) break;

    geometry_msgs::msg::Twist tw;
    tw.angular.z = std::clamp(k_p*err, -wmax, wmax);
    bus_->move_type(tw.angular.z <= 0 ? "rotateRight" : "rotateLeft");
    cmd_vel_pub_->publish(tw);
   // std::cout << "rotating" << std::endl;
  
    rate.sleep();
  }
  std::cout << "rotate done" << std::endl;
  
  geometry_msgs::msg::Twist stop; cmd_vel_pub_->publish(stop);
  bus_->move_type("none");
  robot_->is_moving = false;
}
