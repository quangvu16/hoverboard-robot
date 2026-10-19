// hoverboard_udp_bridge.cpp
//
// ROS 2 side of a hoverboard link that carries wheel telemetry and commands on
// two UDP sockets:
//
//   ESP32 -> here   "HBT1" telemetry, 42-78 bytes, 50 Hz, arrives on telem_port
//   here  -> ESP32  "HBC1" wheel command, 12 bytes, sent at cmd_rate_hz
//
// The ESP32 side is ~/Arduino/ldrobot_ld14p_esp32_mpu9250_ros, which also sends the
// "LDS1" LiDAR scans to UDP 9900 (udp_lidar_receiver_node).
//
// It is deliberately separate from esp_tcp_server_node, which speaks the kaiaai
// length-prefixed JSON protocol over TCP 9000. Run one or the other, never
// both - they would each publish /odom and the odom->base_footprint TF.
//
// The ESP32 sends its telemetry from the same socket it listens for commands
// on, so the reply address is simply the source of the last datagram. No board
// address has to be configured, and a board that reboots onto a new DHCP lease
// is followed automatically. Set esp_ip to pin it down instead.
//
// Wire format, little-endian throughout, from the sketch's own comment block:
//
//   HBT1, ESP32 -> here, 42 bytes
//     0   4  "HBT1"           14  4  ticks_left   int32, accumulated
//     4   2  seq       uint16 18  4  ticks_right  int32, accumulated
//     6   4  millis    uint32 22  2  rpm_left     int16, measured
//    10   2  flags     uint16 24  2  rpm_right    int16
//    12   2  fb_bytes  uint16 26  2  battery      int16, centi-volts
//                             28  2  board_temp   int16, deci-Celsius
//    flags bit0 mainboard link up  30  2  curr_left    int16, centi-amps
//          bit1 command timed out  32  2  curr_right   int16
//          bit2 current reported   34  4  frame_count  uint32
//          bit3 sonar obstacle     38  4  checksum_err uint32
//          bit4 sonar backing off
//          bit5 sonar stale        42  2  sonar_mm     uint16, optional
//          bit8 left obstacle      44  4  sent steer/speed (not used here)
//          bit9 right obstacle     48  2  sonar_left_mm  uint16, optional
//          bit11 left stale        50  2  sonar_right_mm uint16, optional
//          bit12 right stale
//          bit13/14/15 left/right/back LED lit (52-byte firmware only)
//                             52  2  imu_status   uint16, 0 = no IMU
//                             54  8  quat w,x,y,z int16 x4, 1/16384
//                             62  6  gyro x,y,z   int16 x3, 1/16 deg/s
//                             68  6  accel x,y,z  int16 x3, 1/100 m/s^2
//    imu_status bit0 found, bit1 fresh, bit10 compass (NDOF), bits 2-3/4-5/
//               6-7/8-9 calibration 0..3 of sys/gyro/accel/mag
//                             74  1  bat_state    0 unknown 1 ok 2 low 3 critical
//                             75  1  bat_cells
//                             76  2  bat_filtered uint16, centi-volts
//
// Fields were appended over time: 42 bytes from the oldest firmware, 44 with
// the front sonar, 52 with the left/right sonars, 74 with the IMU (MPU9250; BNO055 units), 78 with the
// ESP32's low-battery state (it limits speed / stops by itself). Every
// length is accepted; each topic group is only published when its field is
// present. The IMU fields are in the chip's axes, stamped in imu_frame; the
// URDF's imu_joint rotates them onto base_link.
// The obstacle gate and the back-off / turn-away reflex run on the ESP32
// itself - obstacle_near here just reports the firmware's own verdict.
//
//   HBC1, here -> ESP32, 12 bytes
//     0   4  "HBC1"            6   2  rpm_left   int16, + = forward
//     4   2  seq       uint16  8   2  rpm_right  int16
//                             10   2  flags      uint16, bit0 = enable,
//                                                bit1 = buzzer (/Buzzer)
//
// Wheel ticks arrive as a running total that the ESP32 has already unwrapped,
// so a lost datagram costs nothing: the next one still carries the truth. Only
// a reboot breaks that, which is what the millis field is watched for.
//
// *** Geometry defaults match robot_esp.urdf, which is still TurtleBot3 burger
// (33 mm wheel radius, 140 mm track). A hoverboard base is nothing like that -
// 6.5" wheels are ~82.5 mm radius and the track is several hundred mm. Measure
// the real base and set wheel_radius_m / wheel_separation_m, or odometry will
// be wrong by the ratio between the two. ***

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <cmath>
#include <map>
#include <limits>
#include <cstring>
#include <string>
#include <utility>

namespace {

constexpr uint16_t HBT_LEN = 42;
constexpr uint16_t HBT_LEN_SONAR = 44;
constexpr uint16_t HBT_LEN_SIDE_SONARS = 52;
constexpr uint16_t HBT_LEN_IMU = 74;
constexpr uint16_t HBT_LEN_BATTERY = 78;   // + low-battery state (firmware battery.h)
constexpr uint16_t HBC_LEN = 12;

constexpr uint16_t HBT_FLAG_MAINBOARD_UP = 0x0001;
constexpr uint16_t HBT_FLAG_CMD_TIMEOUT  = 0x0002;
constexpr uint16_t HBT_FLAG_SONAR_NEAR   = 0x0008;
constexpr uint16_t HBT_FLAG_SONAR_STALE  = 0x0020;
constexpr uint16_t HBT_FLAG_SONAR_L_NEAR  = 0x0100;
constexpr uint16_t HBT_FLAG_SONAR_R_NEAR  = 0x0200;
constexpr uint16_t HBT_FLAG_SONAR_L_STALE = 0x0800;
constexpr uint16_t HBT_FLAG_SONAR_R_STALE = 0x1000;
constexpr uint16_t HBT_FLAG_LIGHT_LEFT    = 0x2000;
constexpr uint16_t HBT_FLAG_LIGHT_RIGHT   = 0x4000;
constexpr uint16_t HBT_FLAG_LIGHT_BACK    = 0x8000;
constexpr uint16_t IMU_STATUS_FOUND = 0x0001;
constexpr uint16_t IMU_STATUS_FRESH = 0x0002;
constexpr uint16_t IMU_STATUS_COMPASS = 0x0400;
constexpr uint16_t HBC_FLAG_ENABLE       = 0x0001;
constexpr uint16_t HBC_FLAG_BUZZER       = 0x0002;

inline uint16_t rd_u16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
inline uint32_t rd_u32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline int16_t rd_i16(const uint8_t* p) { return (int16_t)rd_u16(p); }
inline int32_t rd_i32(const uint8_t* p) { return (int32_t)rd_u32(p); }

inline void wr_u16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

class HoverboardUdpBridge : public rclcpp::Node {
public:
  HoverboardUdpBridge() : Node("hoverboard_udp_bridge") {
    telem_port_ = declare_parameter<int>("telem_port", 9902);
    cmd_port_   = declare_parameter<int>("cmd_port", 9901);
    // Empty means "answer whoever sent the telemetry", which is what the
    // sketch expects and needs no static lease.
    esp_ip_     = declare_parameter<std::string>("esp_ip", "");

    wheel_radius_     = declare_parameter<double>("wheel_radius_m", 0.033);
    wheel_separation_ = declare_parameter<double>("wheel_separation_m", 0.14);
    encoder_tpr_      = declare_parameter<double>("encoder_tpr", 90.0);

    max_wheel_mps_ = declare_parameter<double>("max_wheel_speed_mps", 0.5);
    max_rpm_       = declare_parameter<double>("max_wheel_rpm", 1000.0);
    // A hub motor will not start below some wheel speed, and Nav2 routinely
    // asks for less (its small heading corrections are 1-2 RPM). A non-zero
    // command whose faster wheel is below min_wheel_rpm is scaled up, both
    // wheels together so the turn radius is kept; one below
    // wheel_deadband_rpm is treated as zero so controller noise does not get
    // amplified into motion. 0 disables the lift.
    min_rpm_       = declare_parameter<double>("min_wheel_rpm", 0.0);
    deadband_rpm_  = declare_parameter<double>("wheel_deadband_rpm", 0.5);
    invert_left_   = declare_parameter<bool>("invert_left", false);
    invert_right_  = declare_parameter<bool>("invert_right", false);
    // Drive the robot the other way round: the chassis' physical rear is the
    // front. Applied where the wheels meet ROS, after invert_*: the logical
    // left wheel is the physical right one turning the opposite way, so ticks,
    // rpm, /odom, joint states and commands all follow; the side sonars and
    // turn lights swap sides too. The URDF must place the sensors to match
    // (hoverboard.urdf). The firmware's own sonar reflex and light logic still
    // use the physical front.
    reverse_direction_ = declare_parameter<bool>("reverse_direction", false);

    cmd_rate_hz_    = declare_parameter<double>("cmd_rate_hz", 50.0);
    cmd_timeout_s_  = declare_parameter<double>("cmd_timeout_s", 0.5);
    telem_timeout_s_ = declare_parameter<double>("telem_timeout_s", 1.0);
    // A jump larger than this is a counter that restarted, not motion.
    max_tick_step_  = declare_parameter<int>("max_tick_step", 20000);

    odom_frame_  = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_  = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_  = declare_parameter<bool>("publish_tf", true);
    // URDF joint names the wheel angles are published under, so RViz turns the
    // wheels. joint_state_publisher merges these in via its source_list.
    left_joint_  = declare_parameter<std::string>("left_wheel_joint", "wheel_left_joint");
    right_joint_ = declare_parameter<std::string>("right_wheel_joint", "wheel_right_joint");
    // IMU (HBT1 bytes 52-73, BNO055 units; the MPU9250 firmware converts).
    // Diagonal covariances, in the units of sensor_msgs/Imu; robot_localization
    // weighs the IMU against odometry with these.
    imu_frame_     = declare_parameter<std::string>("imu_frame", "imu_link");
    imu_orient_cov_ = declare_parameter<double>("imu_orientation_cov", 0.0025);
    imu_gyro_cov_   = declare_parameter<double>("imu_angular_velocity_cov", 0.0004);
    imu_accel_cov_  = declare_parameter<double>("imu_linear_acceleration_cov", 0.04);
    // MPU9250 firmware: the gyro bias is only known after the robot has stood
    // still ~2 s (imu_status gyro 3). Before that the rate is off by the raw
    // bias, which the EKF would integrate into a slow spin - so hold /imu/data.
    imu_wait_gyro_cal_ = declare_parameter<bool>("imu_wait_gyro_cal", true);
    // The HC-SR04s as sensor_msgs/Range on range/{front,left,right}, for the
    // Nav2 RangeSensorLayer (local costmap) and the collision monitor. No echo
    // (and echoes past range_max) go out as exactly range_max: the costmap
    // layer clears the cone on that, and the collision monitor's points then
    // lie range_max away, outside every zone. A stale (dead) sensor publishes
    // nothing.
    //
    // Not as LaserScans into an ObstacleLayer, though that marks near echoes
    // better: ObstacleLayer goes through tf2_ros::MessageFilter, which on this
    // Humble install (tf2_ros 0.25.23, also 0.25.24) deadlocks against the TF
    // listener - the costmap then silently stops taking ANY sensor data, LiDAR
    // included. Three extra 50 Hz scans made that happen within minutes in
    // testing; RangeSensorLayer does not use the MessageFilter.
    range_fov_ = declare_parameter<double>("range_field_of_view", 0.35);   // rad, HC-SR04 ~20 deg
    range_min_ = declare_parameter<double>("range_min_m", 0.02);
    range_max_ = declare_parameter<double>("range_max_m", 2.0);            // trusted range
    // Stamped this far in the past and sent at most range_rate_hz per sensor.
    // A ping is 60-120 ms old by the time it arrives anyway; stamped "now", each
    // Range had to wait for the next odom TF in the costmap's canTransform, and
    // at 3 x 50 Hz the costmap fell further behind every second (tested).
    range_stamp_lag_ = declare_parameter<double>("range_stamp_lag_s", 0.05);
    range_period_ = 1.0 / std::max(1.0, declare_parameter<double>("range_rate_hz", 10.0));
    // false when twist_mux arbitrates (robot_bringup cmd_mux): then manual
    // driving reaches the wheels only through the mux, never around it.
    subscribe_manual_ = declare_parameter<bool>("subscribe_cmd_vel_manual", true);

    if (encoder_tpr_ <= 0.0 || wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0)
      throw std::runtime_error("encoder_tpr, wheel_radius_m and wheel_separation_m must be > 0");

    m_per_tick_ = (2.0 * M_PI / encoder_tpr_) * wheel_radius_;

    pub_odom_   = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_link_   = create_publisher<std_msgs::msg::String>("hoverboard/link_status", 10);
    pub_batt_   = create_publisher<std_msgs::msg::Float32>("hoverboard/battery_volts", 10);
    // The ESP32's low-battery state, on change only; latched so a late
    // `ros2 topic echo` still shows it. "ok", "low" (speed limited by the
    // ESP32), "critical" (wheels stopped by the ESP32) or "unknown".
    pub_batt_state_ = create_publisher<std_msgs::msg::String>(
      "hoverboard/battery_state", rclcpp::QoS(1).transient_local());
    pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("wheel_joint_states", 10);
    // Same topics and types esp_tcp_server_node publishes, so consumers do not
    // care which base link is running.
    pub_sonar_dist_ = create_publisher<std_msgs::msg::Int32>("ultrasonic/distance_mm", 10);
    pub_sonar_near_ = create_publisher<std_msgs::msg::Bool>("ultrasonic/obstacle_near", 10);
    // The front sensor again under its own namespace, alongside left/right.
    pub_sonar_c_dist_ = create_publisher<std_msgs::msg::Int32>("ultrasonic/center/distance_mm", 10);
    pub_sonar_c_near_ = create_publisher<std_msgs::msg::Bool>("ultrasonic/center/obstacle_near", 10);
    // The side sensors: same message types, one namespace per side.
    pub_sonar_l_dist_ = create_publisher<std_msgs::msg::Int32>("ultrasonic/left/distance_mm", 10);
    pub_sonar_l_near_ = create_publisher<std_msgs::msg::Bool>("ultrasonic/left/obstacle_near", 10);
    pub_sonar_r_dist_ = create_publisher<std_msgs::msg::Int32>("ultrasonic/right/distance_mm", 10);
    pub_sonar_r_near_ = create_publisher<std_msgs::msg::Bool>("ultrasonic/right/obstacle_near", 10);
    // Sensor-data QoS, which is what the range layer and collision monitor use.
    pub_range_f_ = create_publisher<sensor_msgs::msg::Range>("range/front", rclcpp::SensorDataQoS());
    pub_range_l_ = create_publisher<sensor_msgs::msg::Range>("range/left", rclcpp::SensorDataQoS());
    pub_range_r_ = create_publisher<sensor_msgs::msg::Range>("range/right", rclcpp::SensorDataQoS());
    // The ESP32's indicator and stop LEDs as they are lit right now - the
    // indicators toggle with their blink, every 300 ms (LIGHT_BLINK_MS).
    pub_light_left_  = create_publisher<std_msgs::msg::Bool>("lights/left", 10);
    pub_light_right_ = create_publisher<std_msgs::msg::Bool>("lights/right", 10);
    pub_light_back_  = create_publisher<std_msgs::msg::Bool>("lights/back", 10);
    // Only advertised data is fresh fused output; nothing at all is published
    // by an IMU-less firmware or while the chip is missing.
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    // "sys 3 gyro 3 accel 0 mag 0 [compass]", on change only; latched so a
    // late `ros2 topic echo` still shows it.
    pub_imu_calib_ = create_publisher<std_msgs::msg::String>(
      "imu/calib_status", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Best-effort, matching the QoS the rest of this stack subscribes with -
    // a reliable publisher still matches.
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", cmd_qos,
      std::bind(&HoverboardUdpBridge::on_cmd_vel, this, std::placeholders::_1));
    if (subscribe_manual_) {
      sub_cmd_vel_manual_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_manual", cmd_qos,
        std::bind(&HoverboardUdpBridge::on_cmd_vel, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(get_logger(), "cmd_vel_manual not subscribed: commands arrive through the mux");
    }
    // Same topic and type the Yahboom-derived nodes use (hoverboard_ctrl,
    // hoverboard_laser). Carried in every HBC1 frame; the ESP32 silences it by
    // itself if the frames stop.
    sub_buzzer_ = create_subscription<std_msgs::msg::Bool>(
      "Buzzer", 10, [this](const std_msgs::msg::Bool::SharedPtr m) { buzzer_ = m->data; });

    open_socket();

    rx_timer_ = create_wall_timer(std::chrono::milliseconds(2),
      std::bind(&HoverboardUdpBridge::poll_rx, this));
    tx_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_)),
      std::bind(&HoverboardUdpBridge::send_command, this));
    status_timer_ = create_wall_timer(std::chrono::seconds(1),
      std::bind(&HoverboardUdpBridge::publish_status, this));

    RCLCPP_INFO(get_logger(),
      "listening for HBT1 on UDP %d, commands to %s:%d at %.0f Hz",
      telem_port_, esp_ip_.empty() ? "<telemetry sender>" : esp_ip_.c_str(),
      cmd_port_, cmd_rate_hz_);
    RCLCPP_INFO(get_logger(),
      "geometry: wheel_radius %.4f m, wheel_separation %.4f m, %.0f ticks/rev "
      "-> %.6f m/tick", wheel_radius_, wheel_separation_, encoder_tpr_, m_per_tick_);
    RCLCPP_INFO(get_logger(),
      "wheel speed: lift non-zero commands to >= %.1f rpm, deadband %.1f rpm, "
      "cap %.2f m/s", min_rpm_, deadband_rpm_, max_wheel_mps_);
  }

  ~HoverboardUdpBridge() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void open_socket() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("socket() failed");

    int yes = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)telem_port_);

    if (::bind(fd_, (sockaddr*)&addr, sizeof(addr)) != 0) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("bind() on telem_port failed");
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    if (!esp_ip_.empty()) {
      peer_ = sockaddr_in{};
      peer_.sin_family = AF_INET;
      peer_.sin_port = htons((uint16_t)cmd_port_);
      if (::inet_pton(AF_INET, esp_ip_.c_str(), &peer_.sin_addr) != 1)
        throw std::runtime_error("esp_ip is not a valid IPv4 address: " + esp_ip_);
      peer_valid_ = true;
    }
  }

  // ------------------------------------------------------------- telemetry in

  void poll_rx() {
    uint8_t buf[256];
    for (;;) {
      sockaddr_in from{};
      socklen_t from_len = sizeof(from);
      ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, (sockaddr*)&from, &from_len);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        RCLCPP_WARN(get_logger(), "recvfrom error: %d (%s)", errno, strerror(errno));
        return;
      }
      if (n < (ssize_t)HBT_LEN) continue;
      if (std::memcmp(buf, "HBT1", 4) != 0) continue;

      // The sketch sends telemetry from its command socket, so this source is
      // exactly where HBC1 has to go. A configured esp_ip wins.
      if (esp_ip_.empty()) {
        if (!peer_valid_ || peer_.sin_addr.s_addr != from.sin_addr.s_addr) {
          char ip[INET_ADDRSTRLEN] = {0};
          inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
          RCLCPP_INFO(get_logger(), "hoverboard ESP32 at %s:%d", ip, cmd_port_);
        }
        peer_ = from;
        peer_.sin_port = htons((uint16_t)cmd_port_);
        peer_valid_ = true;
      }

      handle_telemetry(buf);
      if (n >= (ssize_t)HBT_LEN_SONAR) handle_sonar(buf);
      if (n >= (ssize_t)HBT_LEN_SIDE_SONARS) {
        handle_side_sonars(buf);
        handle_lights(buf);   // same firmware generation as the side sonars
      }
      if (n >= (ssize_t)HBT_LEN_IMU) handle_imu(buf);
      if (n >= (ssize_t)HBT_LEN_BATTERY) handle_battery(buf);
    }
  }

  void handle_telemetry(const uint8_t* b) {
    const uint32_t esp_ms = rd_u32(b + 6);
    const uint16_t flags  = rd_u16(b + 10);
    // Physical channels with invert_* applied, then mapped onto the logical
    // left/right wheels (reverse_direction).
    const int32_t sl = invert_left_ ? -1 : 1, sr = invert_right_ ? -1 : 1;
    int32_t tl = sl * rd_i32(b + 14);
    int32_t tr = sr * rd_i32(b + 18);
    double rpm_l = sl * (double)rd_i16(b + 22);
    double rpm_r = sr * (double)rd_i16(b + 24);
    if (reverse_direction_) {
      std::swap(tl, tr);
      tl = -tl;
      tr = -tr;
      std::swap(rpm_l, rpm_r);
      rpm_l = -rpm_l;
      rpm_r = -rpm_r;
    }
    const double   batt   = (double)rd_i16(b + 26) / 100.0;

    last_telem_ = now();
    mainboard_up_ = (flags & HBT_FLAG_MAINBOARD_UP) != 0;
    esp_cmd_timeout_ = (flags & HBT_FLAG_CMD_TIMEOUT) != 0;
    fb_bytes_ = rd_u16(b + 12);
    frame_count_ = rd_u32(b + 34);
    checksum_err_ = rd_u32(b + 38);
    battery_volts_ = batt;

    // An ESP32 reboot restarts both the clock and the tick totals.
    if (have_ticks_ && esp_ms < last_esp_ms_) {
      RCLCPP_WARN(get_logger(), "ESP32 rebooted (millis went backwards); re-baselining odometry");
      have_ticks_ = false;
    }
    last_esp_ms_ = esp_ms;

    if (!have_ticks_) {
      last_tl_ = tl;
      last_tr_ = tr;
      have_ticks_ = true;
      return;   // no delta to integrate from a single sample
    }

    const int64_t dtl = (int64_t)tl - (int64_t)last_tl_;
    const int64_t dtr = (int64_t)tr - (int64_t)last_tr_;
    last_tl_ = tl;
    last_tr_ = tr;

    if (std::llabs(dtl) > max_tick_step_ || std::llabs(dtr) > max_tick_step_) {
      RCLCPP_WARN(get_logger(),
        "implausible tick step (%lld/%lld > %d); skipping this sample",
        (long long)dtl, (long long)dtr, max_tick_step_);
      return;
    }

    const double dl = (double)dtl * m_per_tick_;
    const double dr = (double)dtr * m_per_tick_;

    // Actual wheel rotation, wrapped so the double never grows without bound.
    wheel_angle_l_ = std::remainder(wheel_angle_l_ + dl / wheel_radius_, 2.0 * M_PI);
    wheel_angle_r_ = std::remainder(wheel_angle_r_ + dr / wheel_radius_, 2.0 * M_PI);

    const double ds = 0.5 * (dl + dr);
    const double dth = (dr - dl) / wheel_separation_;

    // Integrate about the midpoint heading: over a 20 ms step the difference
    // from an exact arc is far below the encoder resolution.
    const double th_mid = theta_ + 0.5 * dth;
    x_ += ds * std::cos(th_mid);
    y_ += ds * std::sin(th_mid);
    theta_ += dth;
    if (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
    if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // Speeds come from the mainboard's own measured RPM rather than from
    // differentiating ticks - 90 ticks/rev is far too coarse for that.
    const double vl = rpm_l * (2.0 * M_PI / 60.0) * wheel_radius_;
    const double vr = rpm_r * (2.0 * M_PI / 60.0) * wheel_radius_;
    const double vx = 0.5 * (vl + vr);
    const double wz = (vr - vl) / wheel_separation_;

    publish_odom(vx, wz);
    publish_joints(vl / wheel_radius_, vr / wheel_radius_);
  }

  void publish_joints(double wl, double wr) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {left_joint_, right_joint_};
    js.position = {wheel_angle_l_, wheel_angle_r_};
    js.velocity = {wl, wr};
    pub_joints_->publish(js);
  }

  void handle_sonar(const uint8_t* b) {
    const uint16_t flags = rd_u16(b + 10);

    // 0 mm means "no echo within range", and a stale sensor reports nothing
    // at all; both go out as 0, as esp_tcp_server_node does for invalid data.
    std_msgs::msg::Int32 dist;
    dist.data = (flags & HBT_FLAG_SONAR_STALE) ? 0 : (int32_t)rd_u16(b + 42);
    pub_sonar_dist_->publish(dist);
    pub_sonar_c_dist_->publish(dist);

    std_msgs::msg::Bool near;
    near.data = (flags & HBT_FLAG_SONAR_NEAR) != 0;
    pub_sonar_near_->publish(near);
    pub_sonar_c_near_->publish(near);

    if (!(flags & HBT_FLAG_SONAR_STALE)) publish_range(*pub_range_f_, "sonar_link", rd_u16(b + 42));
  }

  // One HC-SR04 reading as sensor_msgs/Range (see range_max_m above).
  void publish_range(rclcpp::Publisher<sensor_msgs::msg::Range>& pub, const char* frame, uint16_t mm) {
    const rclcpp::Time t = now();
    auto& last = range_last_[&pub];
    if (last.nanoseconds() != 0 && (t - last).seconds() < range_period_) return;
    last = t;
    sensor_msgs::msg::Range r;
    r.header.stamp = t - rclcpp::Duration::from_seconds(range_stamp_lag_);
    r.header.frame_id = frame;
    r.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    r.field_of_view = (float)range_fov_;
    r.min_range = (float)range_min_;
    r.max_range = (float)range_max_;
    const double m = mm / 1000.0;
    r.range = (mm == 0 || m >= range_max_) ? (float)range_max_ : (float)std::max(m, range_min_);
    pub.publish(r);
  }

  // Left and right HC-SR04s, same conventions as the front one.
  void handle_side_sonars(const uint8_t* b) {
    const uint16_t flags = rd_u16(b + 10);

    std_msgs::msg::Int32 dist;
    std_msgs::msg::Bool near;

    // reverse_direction: the physical left sonar faces the robot's right.
    auto& pub_l_dist = reverse_direction_ ? pub_sonar_r_dist_ : pub_sonar_l_dist_;
    auto& pub_l_near = reverse_direction_ ? pub_sonar_r_near_ : pub_sonar_l_near_;
    auto& pub_r_dist = reverse_direction_ ? pub_sonar_l_dist_ : pub_sonar_r_dist_;
    auto& pub_r_near = reverse_direction_ ? pub_sonar_l_near_ : pub_sonar_r_near_;

    dist.data = (flags & HBT_FLAG_SONAR_L_STALE) ? 0 : (int32_t)rd_u16(b + 48);
    pub_l_dist->publish(dist);
    near.data = (flags & HBT_FLAG_SONAR_L_NEAR) != 0;
    pub_l_near->publish(near);

    dist.data = (flags & HBT_FLAG_SONAR_R_STALE) ? 0 : (int32_t)rd_u16(b + 50);
    pub_r_dist->publish(dist);
    near.data = (flags & HBT_FLAG_SONAR_R_NEAR) != 0;
    pub_r_near->publish(near);

    // Topic and frame name the robot's side the sensor faces: with
    // reverse_direction the physical left sensor faces the robot's right.
    const char* l_frame = reverse_direction_ ? "sonar_right_link" : "sonar_left_link";
    const char* r_frame = reverse_direction_ ? "sonar_left_link" : "sonar_right_link";
    if (!(flags & HBT_FLAG_SONAR_L_STALE))
      publish_range(reverse_direction_ ? *pub_range_r_ : *pub_range_l_, l_frame, rd_u16(b + 48));
    if (!(flags & HBT_FLAG_SONAR_R_STALE))
      publish_range(reverse_direction_ ? *pub_range_l_ : *pub_range_r_, r_frame, rd_u16(b + 50));
  }

  void handle_lights(const uint8_t* b) {
    const uint16_t flags = rd_u16(b + 10);
    std_msgs::msg::Bool on;
    // reverse_direction: the physical left light is on the robot's right.
    on.data = (flags & HBT_FLAG_LIGHT_LEFT) != 0;
    (reverse_direction_ ? pub_light_right_ : pub_light_left_)->publish(on);
    on.data = (flags & HBT_FLAG_LIGHT_RIGHT) != 0;
    (reverse_direction_ ? pub_light_left_ : pub_light_right_)->publish(on);
    on.data = (flags & HBT_FLAG_LIGHT_BACK) != 0;
    pub_light_back_->publish(on);
  }

  // Bytes 74-77: state, cell count, filtered centi-volts. The ESP32 acts on
  // it by itself (battery.h); this side only reports.
  void handle_battery(const uint8_t* b) {
    static const char* const NAMES[] = {"unknown", "ok", "low", "critical"};
    const uint8_t st = b[74] & 3;
    const double vf = rd_u16(b + 76) / 100.0;
    battery_state_ = NAMES[st];
    if (st == last_batt_state_) return;
    last_batt_state_ = st;
    std_msgs::msg::String m;
    m.data = battery_state_;
    pub_batt_state_->publish(m);
    const unsigned cells = b[75] ? b[75] : 1;
    if (st == 3)
      RCLCPP_ERROR(get_logger(), "battery CRITICAL at %.2f V (%.2f V/cell): the ESP32 has stopped "
                   "the wheels until the pack is charged", vf, vf / cells);
    else if (st == 2)
      RCLCPP_WARN(get_logger(), "battery LOW at %.2f V (%.2f V/cell): the ESP32 limits speed",
                  vf, vf / cells);
    else
      RCLCPP_INFO(get_logger(), "battery %s (%.2f V)", NAMES[st], vf);
  }

  void handle_imu(const uint8_t* b) {
    const uint16_t st = rd_u16(b + 52);

    std::string calib = "none";
    if (st & IMU_STATUS_FOUND) {
      calib = "sys " + std::to_string((st >> 2) & 3) +
              " gyro " + std::to_string((st >> 4) & 3) +
              " accel " + std::to_string((st >> 6) & 3) +
              " mag " + std::to_string((st >> 8) & 3) +
              ((st & IMU_STATUS_COMPASS) ? " compass" : "");
    }
    if (calib != last_imu_calib_) {
      last_imu_calib_ = calib;
      std_msgs::msg::String m;
      m.data = calib;
      pub_imu_calib_->publish(m);
      RCLCPP_INFO(get_logger(), "IMU calibration: %s", calib.c_str());
    }

    if (!(st & IMU_STATUS_FOUND) || !(st & IMU_STATUS_FRESH)) return;
    if (imu_wait_gyro_cal_ && ((st >> 4) & 3) < 3) return;

    constexpr double QUAT = 1.0 / 16384.0;
    constexpr double GYRO = M_PI / 180.0 / 16.0;   // 1/16 deg/s -> rad/s
    constexpr double ACC  = 1.0 / 100.0;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = imu_frame_;
    // 16-bit fixed point leaves |q| off 1 by ~1e-4; normalise so tf2 and
    // robot_localization see a unit quaternion.
    const double qw = rd_i16(b + 54) * QUAT, qx = rd_i16(b + 56) * QUAT,
                 qy = rd_i16(b + 58) * QUAT, qz = rd_i16(b + 60) * QUAT;
    const double qn = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qn < 0.5) return;   // not a rotation: fusion not running yet
    imu.orientation.w = qw / qn;
    imu.orientation.x = qx / qn;
    imu.orientation.y = qy / qn;
    imu.orientation.z = qz / qn;
    imu.angular_velocity.x = rd_i16(b + 62) * GYRO;
    imu.angular_velocity.y = rd_i16(b + 64) * GYRO;
    imu.angular_velocity.z = rd_i16(b + 66) * GYRO;
    imu.linear_acceleration.x = rd_i16(b + 68) * ACC;
    imu.linear_acceleration.y = rd_i16(b + 70) * ACC;
    imu.linear_acceleration.z = rd_i16(b + 72) * ACC;
    for (int i = 0; i < 3; ++i) {
      imu.orientation_covariance[i * 4] = imu_orient_cov_;
      imu.angular_velocity_covariance[i * 4] = imu_gyro_cov_;
      imu.linear_acceleration_covariance[i * 4] = imu_accel_cov_;
    }
    pub_imu_->publish(imu);
  }

  void publish_odom(double vx, double wz) {
    const rclcpp::Time stamp = now();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = vx;
    msg.twist.twist.angular.z = wz;

    // Wheel odometry only: loose on the driven axes, effectively locked on the
    // three a differential drive cannot move in.
    msg.pose.covariance[0]   = 0.01;   // x
    msg.pose.covariance[7]   = 0.01;   // y
    msg.pose.covariance[14]  = 1e6;    // z
    msg.pose.covariance[21]  = 1e6;    // roll
    msg.pose.covariance[28]  = 1e6;    // pitch
    msg.pose.covariance[35]  = 0.05;   // yaw
    msg.twist.covariance[0]  = 0.01;
    msg.twist.covariance[7]  = 1e6;
    msg.twist.covariance[14] = 1e6;
    msg.twist.covariance[21] = 1e6;
    msg.twist.covariance[28] = 1e6;
    msg.twist.covariance[35] = 0.05;
    pub_odom_->publish(msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tfm;
      tfm.header.stamp = stamp;
      tfm.header.frame_id = odom_frame_;
      tfm.child_frame_id = base_frame_;
      tfm.transform.translation.x = x_;
      tfm.transform.translation.y = y_;
      tfm.transform.translation.z = 0.0;
      tfm.transform.rotation = msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tfm);
    }
  }

  // ------------------------------------------------------------- commands out

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vx_ = msg->linear.x;
    cmd_wz_ = msg->angular.z;
    last_cmd_ = now();
    have_cmd_ = true;
  }

  void send_command() {
    if (!peer_valid_) return;   // nothing has announced itself yet

    double rpm_l = 0.0, rpm_r = 0.0;
    bool enable = false;

    // A command that stopped arriving is a dead publisher, a crashed nav2 or a
    // dropped Wi-Fi link. Stop, but keep sending: the frame is also what keeps
    // the ESP32's own safety timeout (and the mainboard's) fed.
    if (have_cmd_ && (now() - last_cmd_).seconds() <= cmd_timeout_s_) {
      double vl = cmd_vx_ - cmd_wz_ * wheel_separation_ * 0.5;
      double vr = cmd_vx_ + cmd_wz_ * wheel_separation_ * 0.5;

      // Scale the pair rather than clipping each wheel, so capping speed never
      // silently changes the commanded turn radius.
      const double max_abs = std::max(std::fabs(vl), std::fabs(vr));
      if (max_abs > max_wheel_mps_ && max_abs > 1e-9) {
        const double scale = max_wheel_mps_ / max_abs;
        vl *= scale;
        vr *= scale;
      }

      const double mps_to_rpm = 60.0 / (2.0 * M_PI * wheel_radius_);
      rpm_l = clampd(vl * mps_to_rpm, -max_rpm_, max_rpm_);
      rpm_r = clampd(vr * mps_to_rpm, -max_rpm_, max_rpm_);
      lift_to_min(rpm_l, rpm_r);
      enable = true;
    }

    // Logical wheels -> physical channels: the reverse of the telemetry mapping.
    if (reverse_direction_) {
      std::swap(rpm_l, rpm_r);
      rpm_l = -rpm_l;
      rpm_r = -rpm_r;
    }
    if (invert_left_)  rpm_l = -rpm_l;
    if (invert_right_) rpm_r = -rpm_r;

    uint8_t buf[HBC_LEN];
    std::memcpy(buf, "HBC1", 4);
    wr_u16(buf + 4, cmd_seq_++);
    wr_u16(buf + 6, (uint16_t)(int16_t)std::lround(rpm_l));
    wr_u16(buf + 8, (uint16_t)(int16_t)std::lround(rpm_r));
    wr_u16(buf + 10, (enable ? HBC_FLAG_ENABLE : 0) | (buzzer_ ? HBC_FLAG_BUZZER : 0));

    if (::sendto(fd_, buf, sizeof(buf), 0, (sockaddr*)&peer_, sizeof(peer_)) < 0) {
      if (++send_fail_ % 100 == 1)
        RCLCPP_WARN(get_logger(), "sendto failed: %d (%s)", errno, strerror(errno));
    }
  }

  void lift_to_min(double& rpm_l, double& rpm_r) const {
    const double big = std::max(std::fabs(rpm_l), std::fabs(rpm_r));
    if (big < deadband_rpm_) {
      rpm_l = rpm_r = 0.0;
      return;
    }
    if (min_rpm_ > 0.0 && big < min_rpm_) {
      const double scale = min_rpm_ / big;
      rpm_l *= scale;
      rpm_r *= scale;
    }
  }

  // ------------------------------------------------------------------ status

  void publish_status() {
    const bool telem_up =
      last_telem_.nanoseconds() != 0 &&
      (now() - last_telem_).seconds() <= telem_timeout_s_;

    std::string s;
    if (!telem_up) {
      s = "waiting: no HBT1 on udp " + std::to_string(telem_port_);
    } else if (!mainboard_up_) {
      // The ESP32 is fine and on the network; its UART to the mainboard is not.
      s = "esp:up mainboard:down frames=" + std::to_string(frame_count_) +
          " cksum_err=" + std::to_string(checksum_err_);
    } else {
      char b[160];
      std::snprintf(b, sizeof(b),
        "esp:up mainboard:up fb=%uB batt=%.2fV%s%s frames=%u cksum_err=%u%s",
        (unsigned)fb_bytes_, battery_volts_,
        battery_state_.empty() ? "" : " batt_state=", battery_state_.c_str(),
        (unsigned)frame_count_,
        (unsigned)checksum_err_, esp_cmd_timeout_ ? " [ESP CMD TIMEOUT]" : "");
      s = b;
    }

    // Log on a change of STATE only. The full string carries frame counts and
    // battery voltage, which change every second, and logging it on every
    // change turned the console into a 1 Hz stream. It is still published on
    // /hoverboard/link_status in full.
    const int state = !telem_up ? 0 : (!mainboard_up_ ? 1 : (esp_cmd_timeout_ ? 3 : 2));
    if (state != last_state_) {
      RCLCPP_INFO(get_logger(), "%s", s.c_str());
      last_state_ = state;
    }
    std_msgs::msg::String m;
    m.data = s;
    pub_link_->publish(m);

    if (telem_up && mainboard_up_) {
      std_msgs::msg::Float32 bv;
      bv.data = (float)battery_volts_;
      pub_batt_->publish(bv);
    }
  }

  // --------------------------------------------------------------- members

  int fd_{-1};
  int telem_port_{9902};
  int cmd_port_{9901};
  std::string esp_ip_;
  sockaddr_in peer_{};
  bool peer_valid_{false};

  double wheel_radius_{0.033};
  double wheel_separation_{0.14};
  double encoder_tpr_{90.0};
  double m_per_tick_{0.0};
  double max_wheel_mps_{0.5};
  double max_rpm_{1000.0};
  double min_rpm_{0.0};
  double deadband_rpm_{0.5};
  bool invert_left_{false};
  bool invert_right_{false};
  bool reverse_direction_{false};

  double cmd_rate_hz_{50.0};
  double cmd_timeout_s_{0.5};
  double telem_timeout_s_{1.0};
  int max_tick_step_{20000};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  bool publish_tf_{true};
  std::string left_joint_{"wheel_left_joint"};
  std::string right_joint_{"wheel_right_joint"};
  std::string imu_frame_{"imu_link"};
  double imu_orient_cov_{0.0025};
  double imu_gyro_cov_{0.0004};
  double imu_accel_cov_{0.04};
  bool imu_wait_gyro_cal_{true};
  double range_fov_{0.35}, range_min_{0.02}, range_max_{2.0};
  double range_stamp_lag_{0.05}, range_period_{0.1};
  std::map<const void*, rclcpp::Time> range_last_;   // per publisher, for range_rate_hz
  bool subscribe_manual_{true};
  std::string last_imu_calib_;

  // Odometry state
  double x_{0.0}, y_{0.0}, theta_{0.0};
  int32_t last_tl_{0}, last_tr_{0};
  bool have_ticks_{false};
  double wheel_angle_l_{0.0}, wheel_angle_r_{0.0};
  uint32_t last_esp_ms_{0};

  // Latest telemetry
  rclcpp::Time last_telem_{0, 0, RCL_ROS_TIME};
  bool mainboard_up_{false};
  bool esp_cmd_timeout_{false};
  uint16_t fb_bytes_{0};
  uint32_t frame_count_{0};
  uint32_t checksum_err_{0};
  double battery_volts_{0.0};
  std::string battery_state_;            // empty = firmware too old to say
  int last_batt_state_{-1};
  int last_state_{-1};

  // Command state
  double cmd_vx_{0.0}, cmd_wz_{0.0};
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
  bool have_cmd_{false};
  uint16_t cmd_seq_{0};
  uint64_t send_fail_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_link_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_batt_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_batt_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_sonar_dist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sonar_near_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_sonar_c_dist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sonar_c_near_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_sonar_l_dist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sonar_l_near_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_sonar_r_dist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sonar_r_near_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range_f_, pub_range_l_, pub_range_r_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_light_left_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_light_right_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_light_back_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_imu_calib_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_manual_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_buzzer_;
  bool buzzer_{false};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr rx_timer_, tx_timer_, status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoverboardUdpBridge>());
  rclcpp::shutdown();
  return 0;
}
