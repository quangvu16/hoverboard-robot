#pragma once
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <cstdint>

namespace esp_tcp_server {

struct ProtocolConfig {
  std::string device_sn;
  double hb_period_s{6.0};
  double state_timeout_s{3.0};
  int hb_timeout_ms_to_esp{15000};
};

class EspProtocol {
public:
  explicit EspProtocol(const ProtocolConfig& cfg);

  void reset();

  // trả true nếu message hợp lệ (đúng Device + có ID/Time) và đã xử lý theo protocol
  bool on_rx_json(const nlohmann::json& msg);

  // nếu cần gửi ack thì trả về JSON cần gửi (Action...)
  std::optional<nlohmann::json> pop_tx_action();

  // gọi từ EspNode: chỉ set acked sau khi SEND RegDeviceAck thành công
  void mark_acked();

  bool acked() const { return acked_; }
  bool got_first_state() const { return got_first_state_; }

  void mark_state_rx(double now_s);
  bool state_timeout(double now_s) const;

private:
  std::string make_id(const std::string& prefix);

private:
  ProtocolConfig cfg_;

  bool acked_{false};
  bool got_first_state_{false};
  double last_state_rx_{0.0};

  uint64_t id_counter_{0};

  // pending action to send (RegDeviceAck)
  std::optional<nlohmann::json> pending_tx_;
};

}  // namespace esp_tcp_server
