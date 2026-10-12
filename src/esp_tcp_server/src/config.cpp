// src/config.cpp
#include "esp_tcp_server/config.hpp"
#include <fstream>
#include <stdexcept>

namespace esp_tcp_server {
nlohmann::json Config::load_json_file(const std::string& path)
{
  std::ifstream f(path);
  if (!f.is_open()) throw std::runtime_error("Cannot open config: " + path);
  nlohmann::json j;
  f >> j;
  return j;
}
}  // namespace esp_tcp_server
