#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <string>

class ROBOT;  // forward

class SLAM {
public:
  explicit SLAM(ROBOT* robot);

  // B?t sub /map và timer g?i map_txt. period_ms c? d?nh 1000ms (có th? d?i trong code).
  void start();
  // T?t sub /map và timer.
  void stop();

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onTimer();
  static std::string first_n_items_semicolon(const std::string& s, size_t n);

  ROBOT* robot_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  // Snapshot map_txt m?i nh?t (chu?i nhi?u dòng có '\n')
  std::string last_map_txt_;
  mutable std::mutex mtx_;

  // K? g?i
  std::chrono::milliseconds period_{1000};
};
