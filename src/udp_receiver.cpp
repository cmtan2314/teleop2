#include "quest_vr/udp_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

#include "quest_vr/logging.hpp"

namespace quest_vr {
namespace {

std::int64_t wallClockNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool parseIpv4(const std::string& host, in_addr& out) {
  return ::inet_pton(AF_INET, host.c_str(), &out) == 1;
}

}  // namespace

UdpJsonReceiver::UdpJsonReceiver(std::string host, std::uint16_t port,
                                 std::size_t recv_buffer_bytes)
    : host_(std::move(host)), port_(port), recv_buffer_bytes_(recv_buffer_bytes) {}

UdpJsonReceiver::~UdpJsonReceiver() { stop(); }

bool UdpJsonReceiver::start() {
  if (thread_.joinable()) {
    logf(LogLevel::kWarn, "start() ignored: receiver thread already running on {}:{}", host_,
         port_);
    return false;
  }
  if (recv_buffer_bytes_ == 0) {
    logf(LogLevel::kError, "start() refused: receive buffer size is 0");
    return false;
  }
  if (port_ == 0) {
    logf(LogLevel::kError, "start() refused: port 0 is not a valid listen port");
    return false;
  }
  in_addr address{};
  if (!parseIpv4(host_, address)) {
    logf(LogLevel::kError, "start() refused: '{}' is not a valid IPv4 address", host_);
    return false;
  }
  running_.store(true);
  thread_ = std::thread(&UdpJsonReceiver::run, this);
  logf(LogLevel::kInfo, "receiver thread started for UDP {}:{}", host_, port_);
  return true;
}

void UdpJsonReceiver::stop() {
  const bool was_running = running_.exchange(false);
  wake_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  if (was_running) {
    logf(LogLevel::kInfo, "receiver thread stopped");
  }
}

std::optional<ReceivedPacket> UdpJsonReceiver::latest() const {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_;
}

std::vector<std::int64_t> UdpJsonReceiver::drainRecvTimestamps() {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<std::int64_t> out(recv_timestamps_.begin(), recv_timestamps_.end());
  recv_timestamps_.clear();
  return out;
}

void UdpJsonReceiver::run() {
  std::vector<char> buffer(recv_buffer_bytes_);
  while (running_.load()) {
    const int fd = openBoundSocket();
    if (fd < 0) {
      // openBoundSocket() already logged the reason.
      waitRetryDelay();
      continue;
    }
    logf(LogLevel::kInfo, "Listening on UDP {}:{}", host_, port_);
    receiveLoop(fd, buffer);
    ::close(fd);
    if (running_.load()) {
      logf(LogLevel::kWarn, "socket closed after an error, reopening in {} ms",
           kSocketRetryDelayMs);
      waitRetryDelay();
    }
  }
}

int UdpJsonReceiver::openBoundSocket() const {
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    logf(LogLevel::kError, "socket() failed: {}", std::strerror(errno));
    return -1;
  }
  const int reuse = 1;
  if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
    // Not fatal: binding can still succeed, only a quick restart may hit EADDRINUSE.
    logf(LogLevel::kWarn, "setsockopt(SO_REUSEADDR) failed: {}", std::strerror(errno));
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (!parseIpv4(host_, address.sin_addr)) {
    logf(LogLevel::kError, "invalid IPv4 host '{}'", host_);
    ::close(fd);
    return -1;
  }
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    logf(LogLevel::kError, "bind({}:{}) failed: {}", host_, port_, std::strerror(errno));
    ::close(fd);
    return -1;
  }
  return fd;
}

void UdpJsonReceiver::receiveLoop(int fd, std::vector<char>& buffer) {
  while (running_.load()) {
    pollfd poll_fd{};
    poll_fd.fd = fd;
    poll_fd.events = POLLIN;
    const int ready = ::poll(&poll_fd, 1, kRecvPollTimeoutMs);
    if (ready == 0) {
      continue;  // timeout: re-check running_ (python: except TimeoutError: continue)
    }
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      logf(LogLevel::kError, "poll() failed: {}", std::strerror(errno));
      return;
    }
    if ((poll_fd.revents & (POLLERR | POLLNVAL)) != 0) {
      logf(LogLevel::kError, "socket reported error (revents=0x{:x})",
           static_cast<unsigned>(poll_fd.revents));
      return;
    }
    if (!drainQueuedDatagrams(fd, buffer)) {
      return;
    }
  }
}

bool UdpJsonReceiver::drainQueuedDatagrams(int fd, std::vector<char>& buffer) {
  std::optional<QuestPacket> last_packet;
  std::int64_t last_recv_ns = 0;
  std::vector<std::int64_t> arrivals;
  bool socket_ok = true;

  // Keep only the freshest packet, but record every accepted packet's arrival.
  while (true) {
    // MSG_TRUNC makes recv() return the real datagram length so truncation is detectable.
    const ssize_t received =
        ::recv(fd, buffer.data(), buffer.size(), MSG_DONTWAIT | MSG_TRUNC);
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;  // queue drained
      }
      if (errno == EINTR) {
        continue;
      }
      logf(LogLevel::kError, "recv() failed: {}", std::strerror(errno));
      socket_ok = false;
      break;
    }
    const std::int64_t recv_ns = wallClockNs();
    const auto length = static_cast<std::size_t>(received);
    if (length > buffer.size()) {
      ++bad_packet_count_;
      if (bad_packet_count_ == 1 || bad_packet_count_ % kBadPacketLogEvery == 0) {
        logf(LogLevel::kWarn, "dropped truncated datagram: {} bytes > buffer {} (bad packets: {})",
             length, buffer.size(), bad_packet_count_);
      }
      continue;
    }
    handleDatagram(std::string_view(buffer.data(), length), recv_ns, last_packet, last_recv_ns,
                   arrivals);
  }

  if (last_packet.has_value() || !arrivals.empty()) {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    for (const std::int64_t stamp : arrivals) {
      if (recv_timestamps_.size() >= kArrivalTimestampCapacity) {
        recv_timestamps_.pop_front();
      }
      recv_timestamps_.push_back(stamp);
    }
    if (last_packet.has_value()) {
      if (sequence_ == 0) {
        logf(LogLevel::kInfo, "first valid packet received");
      }
      latest_ = ReceivedPacket{std::move(*last_packet), ++sequence_, last_recv_ns};
    }
  }
  return socket_ok;
}

void UdpJsonReceiver::handleDatagram(std::string_view text, std::int64_t recv_ns,
                                     std::optional<QuestPacket>& last_packet,
                                     std::int64_t& last_recv_ns,
                                     std::vector<std::int64_t>& arrivals) {
  ParseIssues issues;
  std::optional<QuestPacket> parsed = parseQuestPacket(text, &issues);
  if (!issues.empty()) {
    ++bad_packet_count_;
    if (bad_packet_count_ == 1 || bad_packet_count_ % kBadPacketLogEvery == 0) {
      logf(LogLevel::kWarn, "{} datagram: {} (bad packets: {})",
           parsed.has_value() ? "partially malformed" : "rejected", issues.front(),
           bad_packet_count_);
    }
  }
  if (!parsed.has_value()) {
    return;
  }
  arrivals.push_back(recv_ns);
  last_packet = std::move(parsed);
  last_recv_ns = recv_ns;
}

void UdpJsonReceiver::waitRetryDelay() {
  std::unique_lock<std::mutex> lock(wake_mutex_);
  wake_cv_.wait_for(lock, std::chrono::milliseconds(kSocketRetryDelayMs),
                    [this] { return !running_.load(); });
}

}  // namespace quest_vr
