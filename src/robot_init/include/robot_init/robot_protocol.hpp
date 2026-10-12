#pragma once
#include <vector>
#include <iostream>
#include <string>
#include <optional>
#include <nlohmann/json.hpp>

class ProtocolCodec {
public:
  using Bytes = std::vector<uint8_t>;

  std::vector<uint8_t> build_tcp_message(const nlohmann::json& json_data);

  std::optional<nlohmann::json> extract_data(Bytes& buffer);

  static uint16_t calculate_crc(const Bytes& data);
  static void append_le(Bytes& target, uint32_t value);
  static void append_le(Bytes& target, uint16_t value);

  Bytes build_charge_login_message(const std::string& id);
};
