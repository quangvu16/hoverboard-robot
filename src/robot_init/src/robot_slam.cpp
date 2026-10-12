#include "robot_init/robot_slam.hpp"
#include "robot_init/robot.hpp"

#include <sstream>
#include <iomanip>
#include <sys/socket.h>
#include <fstream>
#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using nlohmann::json;

SLAM::SLAM(ROBOT* robot) : robot_(robot) {}

void SLAM::start() {
  if (map_sub_ || map_timer_) {
    RCLCPP_INFO(robot_->get_logger(), "SLAM already running");
    return;
  }
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_ = robot_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", qos, std::bind(&SLAM::onMap, this, std::placeholders::_1));
  map_timer_ = robot_->create_wall_timer(
    period_, std::bind(&SLAM::onTimer, this));
  RCLCPP_INFO(robot_->get_logger(), "SLAM started: subscribe /map and stream map_txt");
}

void SLAM::stop() {
  if (map_timer_) map_timer_.reset();
  if (map_sub_)   map_sub_.reset();
  RCLCPP_INFO(robot_->get_logger(), "SLAM stopped");
  robot_->is_slam = false;
}

void SLAM::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  const auto& I = msg->info;
  const uint32_t W = I.width, H = I.height;
  if (!W || !H || msg->data.size() != size_t(W*H)) {
    RCLCPP_WARN(robot_->get_logger(), "Invalid map size"); return;
  }
  const double res = I.resolution;
  const double ox = I.origin.position.x;
  const double oy = I.origin.position.y;

  tf2::Quaternion oq(I.origin.orientation.x, I.origin.orientation.y,
                     I.origin.orientation.z, I.origin.orientation.w);
  double r,p,yaw; tf2::Matrix3x3(oq).getRPY(r,p,yaw);
  const double cy = std::cos(yaw), sy = std::sin(yaw);

  // ch?n don v? cho kh?p v?i pose
  constexpr bool kSendMM = false;
  const double S = kSendMM ? 1000.0 : 1.0;

  std::ostringstream out; out.setf(std::ios::fixed);
  out << std::setprecision(kSendMM ? 0 : 3);

  for (uint32_t i=0;i<H;++i){
    for (uint32_t j=0;j<W;++j){
      if (msg->data[i*W + j] == 100){
        // tâm ô trong lu?i
        const double lx = (j + 0.5) * res;
        const double ly = (i + 0.5) * res;
        // -> world (frame 'map'), g?c (0,0) = g?c map
        const double x = ox +  cy*lx - sy*ly;
        const double y = oy +  sy*lx + cy*ly;
        if (kSendMM) out << llround(S*x) << '_' << llround(S*y) << "_0_0_0;";
        else         out << (S*x)        << '_' << (S*y)        << "_0_0_0;";
      }
    }
  }
  std::lock_guard<std::mutex> lk(mtx_);
  last_map_txt_ = out.str();
}

// helper: l?y N m?c d?u, phân tách b?ng ';'
std::string SLAM::first_n_items_semicolon(const std::string& s, size_t n) {
  if (n == 0 || s.empty()) return {};
  size_t pos = 0;
  for (size_t i = 0; i < n; ++i) {
    pos = s.find(';', pos);
    if (pos == std::string::npos) return s;
    ++pos;
  }
  return s.substr(0, pos);
}

void SLAM::onTimer() {
  std::string map_txt;
  { std::lock_guard<std::mutex> lk(mtx_); map_txt = last_map_txt_; }
  if (map_txt.empty()) return;

  nlohmann::json msg = {
    {"Report", "MapData"},
    {"Body", { {"content", map_txt} }},
    {"ID", "12345"}
  };

  robot_->is_slam = true;
  auto framed = robot_->codec_.build_tcp_message(msg);
  robot_->send_all(reinterpret_cast<const uint8_t*>(framed.data()), framed.size());

  RCLCPP_INFO(robot_->get_logger(),
              "map_txt items_total=%zu, bytes=%zu",
              std::count(map_txt.begin(), map_txt.end(), ';'),
              map_txt.size());
  robot_->is_slam = false;
}
