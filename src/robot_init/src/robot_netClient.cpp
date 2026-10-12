#include "robot_init/robot_netClient.hpp"
#include <unistd.h>
#include <cerrno>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <cstring>
#include <chrono>
#include <iostream>

NetClient::NetClient(std::string ip, int port)
: ip_(std::move(ip)), port_(port), sock_fd_(-1) {}

NetClient::~NetClient() { stop(); }

bool NetClient::start(RxCallback cb) {
  if (running_.exchange(true)) return true;
  rx_ = std::move(cb);
  th_ = std::thread(&NetClient::run_loop, this);
  return true;
}

void NetClient::stop() {
  running_ = false;
  close_socket();
  if (th_.joinable()) th_.join();
}

bool NetClient::send_all(const Bytes& d) {
  std::lock_guard<std::mutex> lk(send_mtx_);
  if (sock_fd_ < 0) return false;

  size_t off = 0;
  while (off < d.size()) {
    ssize_t n = ::send(sock_fd_, d.data() + off, d.size() - off, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) continue;
      close_socket();           // quan tr?ng: dóng ngay
      return false;
    }
    if (n == 0) {
      close_socket();
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

bool NetClient::open_and_connect() {
  close_socket();
  sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) return false;

  int one = 1;
  ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

  // Rút ng?n TCP keepalive (Linux)
  #ifdef TCP_KEEPIDLE
    int idle = 10;   ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_KEEPIDLE,  &idle, sizeof(idle));
  #endif
  #ifdef TCP_KEEPINTVL
    int intvl = 5;   ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  #endif
  #ifdef TCP_KEEPCNT
    int cnt = 3;     ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,  sizeof(cnt));
  #endif
  #ifdef TCP_USER_TIMEOUT
    int uto_ms = 15000; // timeout retransmit t?ng, ms
    ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_USER_TIMEOUT, &uto_ms, sizeof(uto_ms));
  #endif
  
    // Timeout d?c/ghi d? tránh block vô h?n
    timeval tv;
    tv.tv_sec = 3; tv.tv_usec = 0;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) != 1) {
      close_socket();
      return false;
    }
    if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      close_socket();
      return false;
    }
  
    connected_ = true;
    if (on_connect_) { try { on_connect_(); } catch (...) {} }
    return true;
}

void NetClient::close_socket() {
  if (sock_fd_ >= 0) {
    ::shutdown(sock_fd_, SHUT_RDWR);
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  if (connected_.exchange(false)) {
    if (on_disconnect_) {
      try { on_disconnect_(); } catch (...) {}
    }
  }
}

void NetClient::run_loop() {
  using namespace std::chrono;
  Bytes buf(4096);
  int backoff_ms = 500;
  auto last_rx = steady_clock::now();

  while (running_) {
    if (!connected_) {
      if (!open_and_connect()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
        backoff_ms = std::min(backoff_ms * 2, 8000);
        continue;
      }
      backoff_ms = 500;
      last_rx = steady_clock::now();
    }

    // recv có SO_RCVTIMEO=3s, nên s? tr? v? -1 v?i EAGAIN/EWOULDBLOCK n?u không có d? li?u
    ssize_t n = ::recv(sock_fd_, buf.data(), buf.size(), 0);
    if (n > 0) {
      last_rx = steady_clock::now();
      if (rx_) rx_(Bytes(buf.begin(), buf.begin() + n));
      continue;
    }
    if (n == 0) {               // peer dóng
      close_socket();
      continue;
    }

    // l?i
    if (errno == EINTR) continue;
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // không có d? li?u quá 10s -> xem nhu link ch?t (Wi-Fi r?t half-open)
      if (steady_clock::now() - last_rx > seconds(10)) {
        close_socket();
      }
      continue;
    }

    // l?i khác
    close_socket();
  }
}
