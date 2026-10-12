// src/esp_node.cpp
#include "esp_tcp_server/esp_node.hpp"
#include <chrono>
#include <cstdio>

namespace esp_tcp_server {
using json = nlohmann::json;

EspNode::EspNode(const json& cfg)
: Node("esp_tcp_server_lenprefix"),
  codec_(cfg.value("max_payload", 6000)),
  protocol_([&]{
    ProtocolConfig p;
    //p.device_sn = cfg.value("device_sn", std::string("CB20608BAK00001"));
    p.hb_period_s = cfg.value("hb_period_s", 6.0);
    p.state_timeout_s = cfg.value("state_timeout_s", 3.0);
    p.hb_timeout_ms_to_esp = cfg.value("hb_timeout_ms_to_esp", 15000);
    return p;
  }()),
  ros_(this)
{
  bind_ip_ = cfg.value("bind_ip", std::string("0.0.0.0"));
  port_    = cfg.value("port", 9000);

  //pcfg_.device_sn = cfg.value("device_sn", std::string("CB20608BAK00001"));
  pcfg_.hb_period_s = cfg.value("hb_period_s", 6.0);
  pcfg_.state_timeout_s = cfg.value("state_timeout_s", 3.0);
  pcfg_.hb_timeout_ms_to_esp = cfg.value("hb_timeout_ms_to_esp", 15000);

  acked_ = false;
  got_first_state_ = false;
  last_state_rx_s_ = 0.0;

  server_.set_on_connect([this](const std::string& a){ on_connect(a); });
  server_.set_on_disconnect([this](const std::string& r){ on_disconnect(r); });
  server_.set_on_bytes([this](const TcpServer::Bytes& b){ on_bytes(b); });

  if (!server_.start(bind_ip_, port_)) {
    throw std::runtime_error("TcpServer start() failed");
  }

  hb_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(pcfg_.hb_period_s),
      std::bind(&EspNode::heartbeat_tick, this));

  timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&EspNode::timeout_tick, this));

  // init throttle timestamp
  last_move_tx_ = this->now();

  // nối callback cmd_vel -> send_move
  ros_.set_send_move_cb([this](float v, float w){
    this->send_move(v, w);
  });

  RCLCPP_INFO(get_logger(), "TCP server listening on %s:%d", bind_ip_.c_str(), port_);
}

EspNode::~EspNode()
{
  server_.stop();
}

int64_t EspNode::now_us_epoch() const
{
  using namespace std::chrono;
  return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

// Drop only current client connection, keep server listening (Python-like)
void EspNode::drop_connection_keep_listen(const std::string& reason)
{
  // publish
  ros_.pub_link("dropped:" + reason);
  RCLCPP_WARN(get_logger(), "Connection dropped: %s", reason.c_str());

  // reset local state
  rx_buf_.clear();
  protocol_.reset();
  acked_ = false;
  got_first_state_ = false;
  last_state_rx_s_ = 0.0;

  // IMPORTANT: close only client (no stop/join, safe in callbacks)
  server_.close_client(reason);
}

void EspNode::on_connect(const std::string& addr)
{
  rx_buf_.clear();
  protocol_.reset();

  acked_ = false;
  got_first_state_ = false;
  last_state_rx_s_ = 0.0;

  ros_.pub_link("connected:" + addr);
  RCLCPP_INFO(get_logger(), "Client connected: %s", addr.c_str());
}

void EspNode::on_disconnect(const std::string& reason)
{
  rx_buf_.clear();
  protocol_.reset();

  acked_ = false;
  got_first_state_ = false;
  last_state_rx_s_ = 0.0;

  ros_.pub_link("dropped:" + reason);
  RCLCPP_WARN(get_logger(), "Connection dropped: %s", reason.c_str());
}

bool EspNode::send_action_json(json j)
{
  j["Time"] = now_us_epoch();
  if (!j.contains("Body")) j["Body"] = json::object();
  j["Device"] = pcfg_.device_sn;

  const std::string s = j.dump();
  std::vector<uint8_t> payload(s.begin(), s.end());
  auto frame = codec_.pack(payload);

  return server_.send_all(frame);
}

void EspNode::on_bytes(const TcpServer::Bytes& chunk)
{
  rx_buf_.insert(rx_buf_.end(), chunk.begin(), chunk.end());

  for (;;) {
    std::optional<std::vector<uint8_t>> payload;

    try {
      payload = codec_.extract(rx_buf_);
    } catch (const std::exception& e) {
      drop_connection_keep_listen(std::string("bad_len:") + e.what());
      return;
    }

    if (!payload) break;

    json msg;
    try {
      msg = json::parse(std::string(payload->begin(), payload->end()));
    } catch (...) {
      continue;
    }

    if (!msg.is_object()) continue;

    const std::string report = msg.value("Report", std::string());

    // ================= RegDevice =================
    // Không cho protocol_ validate RegDevice theo device_sn cũ
    if (report == "RegDevice") {
      active_device_sn_ = msg.value("Device", std::string());

      if (active_device_sn_.empty()) {
        drop_connection_keep_listen("RegDevice_missing_Device");
        return;
      }

      pcfg_.device_sn = active_device_sn_;

      json ack = {
        {"Action", "RegDeviceAck"},
        {"Device", pcfg_.device_sn},
        {"ID", msg.value("ID", std::string("REG_ACK"))},
        {"Body", {
          {"hb_timeout_ms", pcfg_.hb_timeout_ms_to_esp}
        }}
      };

      const bool ok = send_action_json(ack);

      if (!ok) {
        drop_connection_keep_listen("send_RegDeviceAck_failed");
        return;
      }

      acked_ = true;
      got_first_state_ = false;
      last_state_rx_s_ = this->now().seconds();

      ros_.pub_link("acked");

      RCLCPP_INFO(
        get_logger(),
        "RegDeviceAck sent, device=%s",
        active_device_sn_.c_str()
      );

      continue;
    }

    // ================= DeviceState =================
    // Không cho protocol_ validate DeviceState theo device_sn cũ
    if (report == "DeviceState") {
      if (!acked_) continue;

      const std::string dev = msg.value("Device", std::string());
      if (!dev.empty()) {
        active_device_sn_ = dev;
        pcfg_.device_sn = dev;
      }

      got_first_state_ = true;
      last_state_rx_s_ = this->now().seconds();

      ros_.pub_state_json(msg.dump());

      continue;
    }

    // ================= Other reports =================
    // Các gói khác mới đi qua protocol_
    const bool handled = protocol_.on_rx_json(msg);
    if (!handled) continue;

    // Hiện tại ignore các report khác
  }
}

void EspNode::heartbeat_tick()
{
  if (!acked_ || !got_first_state_) return;

  static uint64_t hb_id = 0;
  hb_id++;
  char tmp[32];
  std::snprintf(tmp, sizeof(tmp), "HB_%06llu", (unsigned long long)hb_id);

  json hb = {
    {"Action","HeartBeating"},
    {"Body", json::object()},
    {"ID", std::string(tmp)}
  };

  const bool ok = send_action_json(hb);
  if (!ok) {
    drop_connection_keep_listen("send_HeartBeating_failed");
  }
}

void EspNode::timeout_tick()
{
  if (!acked_ || !got_first_state_) return;

  const double now_s = this->now().seconds();
  if ((now_s - last_state_rx_s_) > pcfg_.state_timeout_s) {
    drop_connection_keep_listen("DeviceState_timeout");
  }
}

bool EspNode::can_tx_to_esp_() const
{
  // chỉ cho gửi khi đã ACK + đã nhận ít nhất 1 DeviceState
  // giống logic heartbeat của bạn
  return acked_ && got_first_state_;
}

void EspNode::send_move(float v, float w)
{
  if (!can_tx_to_esp_()) return;

  // throttle 20Hz
  const auto now = this->now();
  if ((now - last_move_tx_).seconds() < move_tx_period_s_) return;
  last_move_tx_ = now;

  // JSON Action Move gửi xuống ESP
  json j = {
    {"Action","Move"},
    {"Body", {
      {"vx", v},
      {"wz", w}
    }},
    {"ID", "MOVE"}   // dùng id generator có sẵn trong protocol
  };

  const bool ok = send_action_json(j);
  if (!ok) {
    drop_connection_keep_listen("send_Move_failed");
  }
}

}  // namespace esp_tcp_server
