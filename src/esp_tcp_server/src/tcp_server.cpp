// src/tcp_server.cpp
#include "esp_tcp_server/tcp_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace esp_tcp_server {

static void set_timeout_ms(int fd, int ms)
{
  timeval tv{};
  tv.tv_sec = ms / 1000;
  tv.tv_usec = (ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

TcpServer::TcpServer() = default;

TcpServer::~TcpServer()
{
  stop();
}

bool TcpServer::start(const std::string& bind_ip, int port)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (running_) return true;

  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) return false;

  int yes = 1;
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  set_timeout_ms(server_fd_, 1000);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = inet_addr(bind_ip.c_str());

  if (::bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) != 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::listen(server_fd_, 1) != 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  running_ = true;
  th_accept_ = std::thread(&TcpServer::accept_loop, this);
  th_rx_ = std::thread(&TcpServer::rx_loop, this);
  return true;
}

void TcpServer::stop()
{
  // mark not running + close fds
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!running_) return;
    running_ = false;

    if (server_fd_ >= 0) { ::close(server_fd_); server_fd_ = -1; }
    if (conn_fd_ >= 0) { ::close(conn_fd_); conn_fd_ = -1; }
    conn_addr_.clear();
  }

  // avoid joining itself (deadlock)
  const auto self_id = std::this_thread::get_id();

  if (th_accept_.joinable()) {
    if (th_accept_.get_id() == self_id) {
      th_accept_.detach();
    } else {
      th_accept_.join();
    }
  }

  if (th_rx_.joinable()) {
    if (th_rx_.get_id() == self_id) {
      th_rx_.detach();
    } else {
      th_rx_.join();
    }
  }
}

void TcpServer::close_client(const std::string& reason)
{
  std::function<void(const std::string&)> cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (conn_fd_ < 0) return;
    cb = on_disconnect_;
    drop_locked(reason);  // closes fd + clears addr
  }
  // NOTE: drop_locked already calls callback under lock in old code,
  // but we changed to call it here to avoid long work under lock.
  // However we kept original behavior below by letting drop_locked call cb.
  (void)cb;
}

void TcpServer::set_on_connect(std::function<void(const std::string& addr)> cb) { on_connect_ = std::move(cb); }
void TcpServer::set_on_disconnect(std::function<void(const std::string& reason)> cb) { on_disconnect_ = std::move(cb); }
void TcpServer::set_on_bytes(std::function<void(const Bytes& chunk)> cb) { on_bytes_ = std::move(cb); }

bool TcpServer::send_all(const Bytes& data)
{
  // Serialize ALL writes so frames never interleave
  std::lock_guard<std::mutex> txlk(tx_mtx_);

  int fd = -1;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    fd = conn_fd_;
  }
  if (fd < 0) return false;

  const uint8_t* p = data.data();
  size_t left = data.size();
  while (left > 0) {
    ssize_t n = ::send(fd, p, left, 0);
    if (n <= 0) return false;
    p += (size_t)n;
    left -= (size_t)n;
  }
  return true;
}

void TcpServer::drop_locked(const std::string& reason)
{
  if (conn_fd_ >= 0) {
    ::shutdown(conn_fd_, SHUT_RDWR);
    ::close(conn_fd_);
    conn_fd_ = -1;
  }
  conn_addr_.clear();
  if (on_disconnect_) on_disconnect_(reason);
}


void TcpServer::accept_loop()
{
  while (running_) {
    int sfd = -1;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      sfd = server_fd_;
    }
    if (sfd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    sockaddr_in cli{};
    socklen_t sl = sizeof(cli);
    int cfd = ::accept(sfd, (sockaddr*)&cli, &sl);
    if (cfd < 0) {
      // accept timeout or server closed
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    set_timeout_ms(cfd, 500);

    char ip[64]{};
    inet_ntop(AF_INET, &cli.sin_addr, ip, sizeof(ip));
    int cport = ntohs(cli.sin_port);
    std::string addr = std::string(ip) + ":" + std::to_string(cport);

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!running_) {
        ::close(cfd);
        continue;
      }
      if (conn_fd_ >= 0) ::close(conn_fd_);
      conn_fd_ = cfd;
      conn_addr_ = addr;
    }

    if (on_connect_) on_connect_(addr);
  }
}

void TcpServer::rx_loop()
{
  Bytes tmp(4096);

  while (running_) {
    int fd = -1;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      fd = conn_fd_;
    }
    if (fd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    ssize_t n = ::recv(fd, tmp.data(), tmp.size(), 0);
    if (n == 0) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (fd == conn_fd_) drop_locked("peer_closed");
      continue;
    }
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
      std::lock_guard<std::mutex> lk(mtx_);
      if (fd == conn_fd_) drop_locked("recv_error");
      continue;
    }

    if (on_bytes_) {
      Bytes chunk(tmp.begin(), tmp.begin() + n);
      on_bytes_(chunk);  // may call close_client(); safe now
    }
  }
}

}  // namespace esp_tcp_server
