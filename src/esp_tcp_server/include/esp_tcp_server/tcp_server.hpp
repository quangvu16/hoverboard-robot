// include/esp_tcp_server/tcp_server.hpp
#pragma once
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace esp_tcp_server {

class TcpServer {
public:
  using Bytes = std::vector<uint8_t>;

  TcpServer();
  ~TcpServer();

  bool start(const std::string& bind_ip, int port);

  void stop();

  void close_client(const std::string& reason);

  bool send_all(const Bytes& data);

  void set_on_connect(std::function<void(const std::string& addr)> cb);
  void set_on_disconnect(std::function<void(const std::string& reason)> cb);
  void set_on_bytes(std::function<void(const Bytes& chunk)> cb);

private:
  void accept_loop();
  void rx_loop();
  void drop_locked(const std::string& reason);

private:
  int server_fd_{-1};
  int conn_fd_{-1};
  std::string conn_addr_;

  std::mutex mtx_;
  std::mutex tx_mtx_;          // NEW: protect outgoing sends (server->ESP)
  std::atomic<bool> running_{false};

  std::thread th_accept_;
  std::thread th_rx_;

  std::function<void(const std::string&)> on_connect_;
  std::function<void(const std::string&)> on_disconnect_;
  std::function<void(const Bytes&)> on_bytes_;
};

}  // namespace esp_tcp_server
