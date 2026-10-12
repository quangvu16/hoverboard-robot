#include "esp_tcp_server/ros_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>

#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace esp_tcp_server {

static geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();

  return out;
}

static double clamp_d(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

static double ramp_to(double current, double target, double max_delta)
{
  const double diff = target - current;

  if (std::abs(diff) <= max_delta) {
    return target;
  }

  return current + std::copysign(max_delta, diff);
}

static void apply_cmd_accel_decel(double& v, double& w)
{
  static double last_v = 0.0;
  static double last_w = 0.0;
  static auto last_time = std::chrono::steady_clock::now();

  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_time).count();
  last_time = now;

  const double dt_safe = clamp_d(dt, 0.001, 0.2);

  const double ACC_V = 0.25;  // m/s^2
  const double DEC_V = 0.2;  // m/s^2

  const double ACC_W = 1.5;   // rad/s^2
  const double DEC_W = 1.2;   // rad/s^2

  if (!std::isfinite(v) || !std::isfinite(w)) {
    v = 0.0;
    w = 0.0;
    last_v = 0.0;
    last_w = 0.0;
    return;
  }

  const double max_dv =
    (std::abs(v) < std::abs(last_v)) ? DEC_V * dt_safe : ACC_V * dt_safe;

  const double max_dw =
    (std::abs(w) < std::abs(last_w)) ? DEC_W * dt_safe : ACC_W * dt_safe;

  v = ramp_to(last_v, v, max_dv);
  w = ramp_to(last_w, w, max_dw);

  last_v = v;
  last_w = w;
}

static void apply_wheel_min_scale(double& vx, double& wz)
{
  const double WHEEL_BASE_M = 0.138;

  const double V_WHEEL_MAX = 0.15;
  const double V_WHEEL_DEAD = 0.0001;
  const double V_WHEEL_MIN_RUN = 0.025;

  if (!std::isfinite(vx) || !std::isfinite(wz)) {
    vx = 0.0;
    wz = 0.0;
    return;
  }

  double vL = vx - wz * WHEEL_BASE_M * 0.5;
  double vR = vx + wz * WHEEL_BASE_M * 0.5;

  double max_abs = std::max(std::abs(vL), std::abs(vR));

  if (max_abs <= 1e-9) {
    vx = 0.0;
    wz = 0.0;
    return;
  }

  if (max_abs < V_WHEEL_DEAD) {
    vx = 0.0;
    wz = 0.0;
    return;
  }

  if (max_abs < V_WHEEL_MIN_RUN) {
    const double scale = V_WHEEL_MIN_RUN / max_abs;
    vL *= scale;
    vR *= scale;
  }

  max_abs = std::max(std::abs(vL), std::abs(vR));

  if (max_abs > V_WHEEL_MAX) {
    const double scale = V_WHEEL_MAX / max_abs;
    vL *= scale;
    vR *= scale;
  }

  vx = (vR + vL) * 0.5;
  wz = (vR - vL) / WHEEL_BASE_M;

  if (!std::isfinite(vx) || !std::isfinite(wz)) {
    vx = 0.0;
    wz = 0.0;
  }
}

RosBridge::RosBridge(rclcpp::Node* node) : node_(node)
{
  pub_state_ = node_->create_publisher<std_msgs::msg::String>("/esp/device_state", 10);
  pub_link_  = node_->create_publisher<std_msgs::msg::String>("/esp/link_status", 10);

  pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  pub_ultrasonic_distance_ =
    node_->create_publisher<std_msgs::msg::Int32>("/ultrasonic/distance_mm", 10);

  pub_ultrasonic_obstacle_ =
    node_->create_publisher<std_msgs::msg::Bool>("/ultrasonic/obstacle_near", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
    std::bind(&RosBridge::on_cmd_vel, this, std::placeholders::_1)
  );

  sub_manual_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_manual",
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
    std::bind(&RosBridge::on_manual_cmd_vel, this, std::placeholders::_1)
  );
}

void RosBridge::pub_state_json(const std::string& s)
{
  {
    std_msgs::msg::String m;
    m.data = s;
    pub_state_->publish(m);
  }

  try {
    const auto j = nlohmann::json::parse(s);
    if (!j.is_object()) return;

    if (j.value("Report", std::string()) != "DeviceState") return;

    if (!j.contains("Body") || !j["Body"].is_object()) return;
    const auto& body = j["Body"];

    // ================= HC-SR04 =================
    if (body.contains("ultrasonic") && body["ultrasonic"].is_object()) {
      const auto& us = body["ultrasonic"];

      ultrasonic_valid_ = us.value("valid", false);
      ultrasonic_distance_mm_ = us.value("distance_mm", 0);

      // Server tự quyết định obstacle_near
      const bool near_now =
        ultrasonic_valid_ &&
        ultrasonic_distance_mm_ > 0 &&
        ultrasonic_distance_mm_ <= ultrasonic_stop_mm_;

      if (near_now) {
        ultrasonic_near_count_++;
        ultrasonic_far_count_ = 0;
      } else {
        ultrasonic_far_count_++;
        ultrasonic_near_count_ = 0;
      }

      if (ultrasonic_near_count_ >= 2) {
        obstacle_near_ = true;
      }

      if (ultrasonic_far_count_ >= 3) {
        obstacle_near_ = false;
      }

      std_msgs::msg::Int32 dist_msg;
      dist_msg.data = ultrasonic_distance_mm_;
      pub_ultrasonic_distance_->publish(dist_msg);

      std_msgs::msg::Bool obs_msg;
      obs_msg.data = obstacle_near_;
      pub_ultrasonic_obstacle_->publish(obs_msg);

      if (obstacle_near_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          500,
          "HC-SR04 obstacle near: distance=%d mm, stop_mm=%d",
          ultrasonic_distance_mm_,
          ultrasonic_stop_mm_
        );

        if (send_move_cb_) {
          send_move_cb_(0.0f, 0.0f);
        }
      }
    } else {
      ultrasonic_valid_ = false;
      obstacle_near_ = false;
      ultrasonic_distance_mm_ = 0;

      std_msgs::msg::Int32 dist_msg;
      dist_msg.data = 0;
      pub_ultrasonic_distance_->publish(dist_msg);

      std_msgs::msg::Bool obs_msg;
      obs_msg.data = false;
      pub_ultrasonic_obstacle_->publish(obs_msg);
    }
    // END HCSR04

    if (!body.contains("odom") || !body["odom"].is_object()) return;
    const auto& od = body["odom"];

    if (!od.contains("x") || !od.contains("y") || !od.contains("yaw") ||
        !od.contains("v") || !od.contains("w")) {
      return;
    }

    const double x   = od["x"].get<double>();
    const double y   = od["y"].get<double>();
    const double yaw = od["yaw"].get<double>();
    const double v   = od["v"].get<double>();
    const double w   = od["w"].get<double>();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id  = base_frame_;

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = quat_from_yaw(yaw);

    msg.twist.twist.linear.x  = v;
    msg.twist.twist.linear.y  = 0.0;
    msg.twist.twist.linear.z  = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = w;

    pub_odom_->publish(msg);

    geometry_msgs::msg::TransformStamped tfm;
    tfm.header.stamp = msg.header.stamp;
    tfm.header.frame_id = odom_frame_;
    tfm.child_frame_id  = base_frame_;

    tfm.transform.translation.x = x;
    tfm.transform.translation.y = y;
    tfm.transform.translation.z = 0.0;
    tfm.transform.rotation = msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tfm);
  }
  catch (...) {
    return;
  }
}

void RosBridge::pub_link(const std::string& s)
{
  std_msgs::msg::String m;
  m.data = s;
  pub_link_->publish(m);
}

void RosBridge::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!send_move_cb_) return;

  if (obstacle_near_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      500,
      "CMD_NAV blocked by HC-SR04: distance=%d mm, stop_mm=%d",
      ultrasonic_distance_mm_,
      ultrasonic_stop_mm_
    );

    send_move_cb_(0.0f, 0.0f);
    return;
  }

  double v_raw = msg->linear.x;
  double w_raw = msg->angular.z;

  double v = v_raw;
  double w = w_raw;

  apply_wheel_min_scale(v, w);
  //apply_cmd_accel_decel(v, w);

  std::cout << "[CMD_NAV] raw=("
            << v_raw << ", " << w_raw
            << ") out=(" << v << ", " << w << ")"
            << std::endl;

  send_move_cb_(static_cast<float>(v), static_cast<float>(w));
}

void RosBridge::on_manual_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!send_move_cb_) return;

  if (obstacle_near_ && msg->linear.x > 0.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      500,
      "CMD_MANUAL forward blocked by HC-SR04: distance=%d mm, stop_mm=%d",
      ultrasonic_distance_mm_,
      ultrasonic_stop_mm_
    );

    send_move_cb_(0.0f, 0.0f);
    return;
  }

  double v = clamp_d(msg->linear.x, -0.20, 0.20);
  double w = clamp_d(msg->angular.z, -2.00, 2.00);

  std::cout << "[CMD_MANUAL] out=("
            << v << ", " << w << ")"
            << std::endl;

  send_move_cb_(static_cast<float>(v), static_cast<float>(w));
}

}  // namespace esp_tcp_server
