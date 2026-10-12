// include/esp_tcp_server/esp_node.hpp (UPDATED for fixed esp_node.cpp)
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <vector>

#include "esp_tcp_server/lenprefix_codec.hpp"
#include "esp_tcp_server/tcp_server.hpp"
#include "esp_tcp_server/esp_protocol.hpp"
#include "esp_tcp_server/ros_bridge.hpp"

namespace esp_tcp_server {

class EspNode : public rclcpp::Node {
public:
  explicit EspNode(const nlohmann::json& cfg);
  ~EspNode() override;

private:
  int64_t now_us_epoch() const;

  bool can_tx_to_esp_() const;
  void send_move(float v, float w);


  // TcpServer callbacks
  void on_connect(const std::string& addr);
  void on_disconnect(const std::string& reason);
  void on_bytes(const TcpServer::Bytes& chunk);

  // timers
  void heartbeat_tick();
  void timeout_tick();

  // send (returns success so we can set acked_ like Python)
  bool send_action_json(nlohmann::json j);

  // drop connection but keep listening (Python-like behavior)
  void drop_connection_keep_listen(const std::string& reason);

private:
  // config
  std::string bind_ip_;
  int port_{9000};
  std::string active_device_sn_;
  ProtocolConfig pcfg_;

  // modules
  LenPrefixCodec codec_;
  TcpServer server_;
  EspProtocol protocol_;
  RosBridge ros_;

  // rx buffer
  std::vector<uint8_t> rx_buf_;

  // timers
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // ---- runtime state (match Python semantics) ----
  bool acked_{false};
  bool got_first_state_{false};
  double last_state_rx_s_{0.0};

  // throttle gửi Move (tránh spam 50-100Hz)
  rclcpp::Time last_move_tx_;
  double move_tx_period_s_ = 0.03; // 33Hz

};

}  // namespace esp_tcp_server
