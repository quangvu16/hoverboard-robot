// include/esp_tcp_server/config.hpp
#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace esp_tcp_server {
struct Config {
  static nlohmann::json load_json_file(const std::string& path);
};
}  // namespace esp_tcp_server
