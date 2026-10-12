// udp_lidar_receiver.cpp (full, "chuẩn bản chất": flip bằng angle_increment âm, không đảo mảng)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <cmath>
#include <limits>

static inline uint16_t rd_u16_le(const uint8_t* p){
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd_u32_le(const uint8_t* p){
  return (uint32_t)p[0]
      | ((uint32_t)p[1] << 8)
      | ((uint32_t)p[2] << 16)
      | ((uint32_t)p[3] << 24);
}

class UdpLidarReceiver : public rclcpp::Node {
public:
  UdpLidarReceiver() : Node("udp_lidar_receiver") {
    port_      = declare_parameter<int>("port", 9900);
    frame_id_  = declare_parameter<std::string>("frame_id", "base_scan");

    // NEW:
    // - flip_lr=true: dữ liệu từ ESP đang theo chiều CW (mirror trái/phải khi publish kiểu CCW)
    //   -> sửa "đúng bản chất" bằng angle_increment âm
    // - angle0_deg: offset để quay toàn bộ scan (nếu 0° của ESP không trùng "phía trước" ROS)
    flip_lr_    = declare_parameter<bool>("flip_lr", true);
    angle0_deg_ = declare_parameter<double>("angle0_deg", 180.0);

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    open_socket_();
    timer_ = create_wall_timer(std::chrono::milliseconds(5),
                               std::bind(&UdpLidarReceiver::poll_, this));

    RCLCPP_INFO(get_logger(),
      "Listening UDP port %d, publishing /scan frame_id=%s flip_lr=%s angle0_deg=%.2f",
      port_, frame_id_.c_str(), flip_lr_ ? "true" : "false", angle0_deg_);
  }

  ~UdpLidarReceiver() override { if (fd_ >= 0) ::close(fd_); }

private:
  void open_socket_() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("socket() failed");

    int yes = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port_);

    if (::bind(fd_, (sockaddr*)&addr, sizeof(addr)) != 0) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("bind() failed");
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  static inline float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= (float)(2.0 * M_PI);
    while (a < -(float)M_PI) a += (float)(2.0 * M_PI);
    return a;
  }

  void poll_() {
    uint8_t buf[4096];

    for (;;) {
      ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, nullptr, nullptr);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        RCLCPP_WARN(get_logger(), "recvfrom error: %d (%s)", errno, strerror(errno));
        return;
      }

      if (n < 14) continue;
      if (!(buf[0]=='L' && buf[1]=='D' && buf[2]=='S' && buf[3]=='1')) continue;

      uint16_t count = rd_u16_le(buf + 4);
      if (count == 0) continue;

      size_t need = 14 + (size_t)count * 2;
      if ((size_t)n < need) continue;

      uint32_t scan_id = rd_u32_le(buf + 6);
      (void)scan_id;

      sensor_msgs::msg::LaserScan scan;
	rclcpp::Time stamp = now();

	scan.header.stamp = stamp;
	scan.header.frame_id = frame_id_;

	if (has_last_scan_stamp_) {
	  double dt = (stamp - last_scan_stamp_).seconds();

	  if (dt > 0.001 && dt < 1.0) {
	    scan.scan_time = dt;
	  } else {
	    scan.scan_time = 0.1;
	  }
	} else {
	  scan.scan_time = 0.1;
	  has_last_scan_stamp_ = true;
	}

	last_scan_stamp_ = stamp;
	// 0, not scan_time / count: the stamp is the UDP arrival time, which
	// jitters by ~25 ms, so spreading the points over scan_time made
	// consecutive scans overlap in time and Cartographer dropped the overlap
	// ("Dropped N earlier points"). All points share the stamp instead.
	scan.time_increment = 0.0f;

      scan.range_min = 0.02f;
      scan.range_max = 20.0f;
      scan.ranges.resize(count);

      const uint8_t* p = buf + 14;

      // Không đảo mảng: index i giữ nguyên đúng như ESP gửi (bin i)
      for (uint16_t i = 0; i < count; i++) {
        uint16_t mm = rd_u16_le(p + i*2);
	if (mm == 0) {
	  scan.ranges[i] = std::numeric_limits<float>::infinity();
	} else {
	  scan.ranges[i] = (float)mm / 1000.0f;
	}
      }

      // Góc theo chuẩn ROS: 0 rad là phía trước, dải [-pi..+pi]
      // "Flip bản chất": nếu ESP bin tăng theo CW, dùng angle_increment âm.
      const float inc_mag = (float)(2.0 * M_PI / (double)count);
      const float inc = flip_lr_ ? -inc_mag : inc_mag;

      // Offset quay toàn bộ scan (deg), dương = CCW
      const float off = (float)(angle0_deg_ * M_PI / 180.0);

      // Chọn angle_min sao cho vẫn bao phủ quanh 0 rad
      // - Với inc dương: angle_min = -pi + off
      // - Với inc âm:    angle_min = +pi + off (để angle_max đi về -pi)
      scan.angle_increment = inc;
      scan.angle_min = wrap_pi((flip_lr_ ? (float)M_PI : (float)-M_PI) + off);
      scan.angle_max = scan.angle_min + scan.angle_increment * (count - 1);

      pub_scan_->publish(scan);
    }
  }

  int port_{9900};
  int fd_{-1};
  std::string frame_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

  bool flip_lr_{true};
  double angle0_deg_{0.0};
  
  rclcpp::Time last_scan_stamp_;
  bool has_last_scan_stamp_{false};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpLidarReceiver>());
  rclcpp::shutdown();
  return 0;
}
