#include "esp_tcp_server/esp_protocol.hpp"
#include <cstdio>   // snprintf

namespace esp_tcp_server {
using json = nlohmann::json;

EspProtocol::EspProtocol(const ProtocolConfig& cfg) : cfg_(cfg) {}

void EspProtocol::reset()
{
  acked_ = false;
  got_first_state_ = false;
  last_state_rx_ = 0.0;
  pending_tx_.reset();
  id_counter_ = 0;
}

std::string EspProtocol::make_id(const std::string& prefix)
{
  id_counter_++;
  char tmp[64];
  std::snprintf(tmp, sizeof(tmp), "%s_%06llu",
                prefix.c_str(),
                static_cast<unsigned long long>(id_counter_));
  return std::string(tmp);
}

bool EspProtocol::on_rx_json(const json& msg)
{
  if (!msg.is_object()) return false;

  // ignore ESP HeartBeating report (như python)
  if (msg.value("Report", std::string()) == "HeartBeating") return true;

  // validate minimal
  if (!msg.contains("Report")) return false;
  if (msg.value("Device", std::string()) != cfg_.device_sn) return false;
  if (!msg.contains("ID") || !msg.contains("Time")) return false;

  const std::string report = msg.value("Report", "");

  if (report == "RegDevice") {
    // Nếu đã ACK rồi thì bỏ qua
    if (acked_) return true;

    // Nếu đã có pending ack (đã chuẩn bị nhưng chưa gửi thành công) thì cũng không tạo lại
    if (pending_tx_.has_value()) return true;

    json body = {
      {"ok", true},
      {"hb_timeout_ms", cfg_.hb_timeout_ms_to_esp},
      {"state_timeout_s", cfg_.state_timeout_s}
    };

    pending_tx_ = json{
      {"Action","RegDeviceAck"},
      {"Body", body},
      {"Device", cfg_.device_sn},
      {"ID", make_id("ACK")}
      // Time: để EspNode điền cho thống nhất
    };

    // IMPORTANT: KHÔNG set acked_ ở đây
    return true;
  }

  if (report == "DeviceState") {
    // Chỉ đánh dấu got_first_state nếu đã acked (đúng python)
    if (acked_) {
      got_first_state_ = true;
    }
    return true;
  }

  return true;
}

std::optional<json> EspProtocol::pop_tx_action()
{
  auto out = pending_tx_;
  pending_tx_.reset();
  return out;
}

void EspProtocol::mark_acked()
{
  acked_ = true;
  got_first_state_ = false;
  last_state_rx_ = 0.0;
}

void EspProtocol::mark_state_rx(double now_s)
{
  got_first_state_ = true;
  last_state_rx_ = now_s;
}

bool EspProtocol::state_timeout(double now_s) const
{
  if (!acked_ || !got_first_state_) return false;
  return (now_s - last_state_rx_) > cfg_.state_timeout_s;
}

}  // namespace esp_tcp_server
