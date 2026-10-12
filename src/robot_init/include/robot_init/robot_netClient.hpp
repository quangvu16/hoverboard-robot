#pragma once
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>

class NetClient {
public:
  using Bytes = std::vector<uint8_t>;
  using RxCallback = std::function<void(const Bytes&)>;
  using ConnectCallback = std::function<void()>;
  using DisconnectCallback = std::function<void()>;

  NetClient(std::string ip, int port);
  ~NetClient();

  bool start(RxCallback cb);
  void stop();
  bool send_all(const Bytes& d);
  bool connected() const { return connected_.load(); }

  void set_on_connect(ConnectCallback cb)       { on_connect_ = std::move(cb); }
  void set_on_disconnect(DisconnectCallback cb) { on_disconnect_ = std::move(cb); }

private:
  bool open_and_connect();
  void close_socket();
  void run_loop();

  std::string ip_;
  int port_;
  int sock_fd_;
  std::thread th_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  RxCallback rx_;
  ConnectCallback on_connect_;
  DisconnectCallback on_disconnect_;   // <-- thêm
  std::mutex send_mtx_;
};

