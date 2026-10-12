#include "robot_init/robot_protocol.hpp"
#include <cstdint>

using Bytes = ProtocolCodec::Bytes;
using namespace std;

uint16_t ProtocolCodec::calculate_crc(const Bytes& data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

void ProtocolCodec::append_le(Bytes& target, uint32_t value) {
  target.push_back(value & 0xFF);
  target.push_back((value >> 8) & 0xFF);
  target.push_back((value >> 16) & 0xFF);
  target.push_back((value >> 24) & 0xFF);
}

void ProtocolCodec::append_le(Bytes& target, uint16_t value) {
  target.push_back(value & 0xFF);
  target.push_back((value >> 8) & 0xFF);
}

Bytes ProtocolCodec::build_tcp_message(const nlohmann::json& data) {
      // Constants
    const uint8_t STX = 0x02;
    const uint8_t DIR = 0x56;
    const uint16_t RESERVED = 0x0000;
    const uint16_t VERSION = 0x0000;
    const std::vector<uint8_t> EXTENDED(16, 0x00);
    const std::vector<uint8_t> MSG_TYPE = {0x00, 0x00};
    const uint8_t NEWLINE = 0x0A;
    const uint8_t ETX = 0x03;

    // === Step 1: Generate compact JSON string (no whitespace)
    std::string json_str = data.dump();
    std::string compact_json;
    for (char c : json_str)
        if (c != ' ' && c != '\n') compact_json += c;

    // === Step 2: Message body = MSG_TYPE + JSON + NEWLINE
    std::vector<uint8_t> body(MSG_TYPE);
    body.insert(body.end(), compact_json.begin(), compact_json.end());
    body.push_back(NEWLINE);

    // === Step 3: Calculate message body CRC (big-endian)
    uint16_t crc_body = calculate_crc(body);
    body.push_back((crc_body >> 8) & 0xFF);  // High byte first
    body.push_back(crc_body & 0xFF);         // Low byte

    // === Step 4: Calculate message length (JSON + type + newline only)
    uint32_t length = MSG_TYPE.size() + compact_json.size() + 1;  // Do not include CRC or ETX

    // === Step 5: Construct header
    std::vector<uint8_t> header;
    header.push_back(STX);
    header.push_back(DIR);
    append_le(header, RESERVED);
    append_le(header, length);  // Little-endian
    append_le(header, VERSION);
    header.insert(header.end(), EXTENDED.begin(), EXTENDED.end());

    // === Step 6: Calculate CRC of header (little-endian)
    uint16_t crc_header = calculate_crc(header);
    header.push_back(crc_header & 0xFF);         // low byte
    header.push_back((crc_header >> 8) & 0xFF);  // high byte

    // === Step 7: Final message = header + body + ETX
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), header.begin(), header.end());
    packet.insert(packet.end(), body.begin(), body.end());
    packet.push_back(ETX);
    
  return packet; // tr? v? dúng packet theo giao th?c c?a b?n
}

std::optional<nlohmann::json> ProtocolCodec::extract_data(Bytes& buffer) {
    // Step 1: Align to valid header start (0x02 0x56)
    while (buffer.size() >= 2) {
        if (buffer[0] == 0x02 && buffer[1] == 0x56) break;
        buffer.erase(buffer.begin());
    }

    // Step 2: Check for minimum header size
    if (buffer.size() < 26) return std::nullopt;

    // Step 3: Read body length (4 bytes at offset 4)
    uint32_t body_len = 0;
    std::memcpy(&body_len, &buffer[4], sizeof(uint32_t));

    // Step 4: Check if full message has arrived
    size_t total_len = 26 + body_len + 2 + 1;  // header + body + CRC + ETX
    if (buffer.size() < total_len) return std::nullopt;

    // Step 5: Debug full message bytes
    // std::cout << "[DEBUG] extract_data: Full message size = " << total_len << "\n";
    // std::cout << "[DEBUG] extract_data: Raw message (first 32 bytes): ";
    // for (size_t i = 0; i < std::min<size_t>(32, buffer.size()); ++i) {
    //     printf("%02X ", buffer[i]);
    // }
    // std::cout << std::endl;

    // Step 6: Extract JSON string
    const char* body_start = reinterpret_cast<const char*>(&buffer[26 + 2]);  // skip 2-byte type
    const size_t json_max_len = body_len - 2 + 1;  // include final '\n'

    std::string_view raw_body(body_start, json_max_len);

    // Step 7: Find JSON block by counting braces
    size_t start = raw_body.find('{');
    if (start == std::string::npos) {
        std::cerr << "âŒ extract_data: No '{' found\n";
        buffer.erase(buffer.begin(), buffer.begin() + total_len);
        return std::nullopt;
    }

    int brace_count = 0;
    size_t end = std::string::npos;
    for (size_t i = start; i < raw_body.size(); ++i) {
        if (raw_body[i] == '{') ++brace_count;
        else if (raw_body[i] == '}') {
            --brace_count;
            if (brace_count == 0) {
                end = i;
                break;
            }
        }
    }

    if (end == std::string::npos) {
        std::cerr << "âŒ extract_data: No matching '}' found\n";
        buffer.erase(buffer.begin(), buffer.begin() + total_len);
        return std::nullopt;
    }

    std::string json_str = std::string(raw_body.substr(start, end - start + 1));
    //std::cout << "[DEBUG] extract_data: Extracted JSON = " << json_str << std::endl;

    // Step 8: Parse and return JSON
    try {
        auto parsed = nlohmann::json::parse(json_str);
        buffer.erase(buffer.begin(), buffer.begin() + total_len);
        return parsed;
    } catch (const std::exception& e) {
        std::cerr << "âŒ extract_data: JSON parse failed: " << e.what() << "\n";
        std::cerr << "âŒ extract_data: Raw JSON: " << json_str << std::endl;
        buffer.erase(buffer.begin(), buffer.begin() + total_len);
        return std::nullopt;
    }
}

Bytes ProtocolCodec::build_charge_login_message(const std::string& id) {
  // TODO: n?u b?n có hàm này trong code g?c thì copy thân vào dây
  return {};
}
