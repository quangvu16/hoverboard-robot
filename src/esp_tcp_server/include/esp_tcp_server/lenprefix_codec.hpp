// include/esp_tcp_server/lenprefix_codec.hpp
#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace esp_tcp_server {

class LenPrefixCodec {
public:
  explicit LenPrefixCodec(size_t max_payload = 6000);

  std::vector<uint8_t> pack(const std::vector<uint8_t>& payload) const;

  // extract 1 payload (bytes) from buffer if ?? d? li?u
  // tr? nullopt n?u ch?a ??
  // throw runtime_error n?u bad_len
  std::optional<std::vector<uint8_t>> extract(std::vector<uint8_t>& buffer) const;

private:
  size_t max_payload_;
};

}  // namespace esp_tcp_server
