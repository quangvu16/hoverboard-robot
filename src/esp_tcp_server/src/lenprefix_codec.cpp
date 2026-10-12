// src/lenprefix_codec.cpp
#include "esp_tcp_server/lenprefix_codec.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>

namespace esp_tcp_server {

LenPrefixCodec::LenPrefixCodec(size_t max_payload) : max_payload_(max_payload) {}

std::vector<uint8_t> LenPrefixCodec::pack(const std::vector<uint8_t>& payload) const
{
  uint32_t n = static_cast<uint32_t>(payload.size());
  uint32_t be = htonl(n);

  std::vector<uint8_t> out(4 + payload.size());
  std::memcpy(out.data(), &be, 4);
  if (!payload.empty()) std::memcpy(out.data() + 4, payload.data(), payload.size());
  return out;
}

std::optional<std::vector<uint8_t>> LenPrefixCodec::extract(std::vector<uint8_t>& buffer) const
{
  if (buffer.size() < 4) return std::nullopt;

  uint32_t be = 0;
  std::memcpy(&be, buffer.data(), 4);
  uint32_t n = ntohl(be);

  if (n == 0 || n > max_payload_) {
    throw std::runtime_error("bad_len:" + std::to_string(n));
  }
  if (buffer.size() < 4 + n) return std::nullopt;

  std::vector<uint8_t> payload(n);
  std::memcpy(payload.data(), buffer.data() + 4, n);
  buffer.erase(buffer.begin(), buffer.begin() + 4 + n);
  return payload;
}

}  // namespace esp_tcp_server
