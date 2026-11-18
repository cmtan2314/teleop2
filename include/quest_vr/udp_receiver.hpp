// Port of udp_receiver.py: one background thread binds a UDP socket, drains
// every queued datagram and keeps only the freshest parsed packet. Consumers
// (the processing / IK thread) poll latest() at their own rate.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "quest_vr/constants.hpp"
#include "quest_vr/packet.hpp"

namespace quest_vr {

struct ReceivedPacket {
  QuestPacket packet;
  // Increments on every accepted datagram. Lets a consumer tell a fresh packet
  // from the same one re-read on the next tick (python cannot: latest() just
  // returns the same dict again).
  std::uint64_t sequence = 0;
  std::int64_t recv_time_ns = 0;  // wall clock (system_clock), like time.time_ns()
};

class UdpJsonReceiver {
 public:
  UdpJsonReceiver(std::string host, std::uint16_t port,
                  std::size_t recv_buffer_bytes = kDefaultRecvBufferBytes);
  ~UdpJsonReceiver();

  UdpJsonReceiver(const UdpJsonReceiver&) = delete;
  UdpJsonReceiver& operator=(const UdpJsonReceiver&) = delete;

  // Validates the configuration and starts the receiver thread.
  // Returns false (and logs why) if the config is invalid or already started.
  bool start();
  // Stops and joins the thread. Safe to call more than once.
  void stop();

  // Freshest packet received so far, or nullopt if none yet.
  std::optional<ReceivedPacket> latest() const;
  // Arrival timestamps (ns) of every accepted packet since the previous call.
  std::vector<std::int64_t> drainRecvTimestamps();

 private:
  void run();
  int openBoundSocket() const;
  // Receives until stop() or a socket error. Returns when the socket must be reopened.
  void receiveLoop(int fd, std::vector<char>& buffer);
  // Drains every datagram currently queued on fd. Returns false on a socket error.
  bool drainQueuedDatagrams(int fd, std::vector<char>& buffer);
  void handleDatagram(std::string_view text, std::int64_t recv_ns,
                      std::optional<QuestPacket>& last_packet, std::int64_t& last_recv_ns,
                      std::vector<std::int64_t>& arrivals);
  void waitRetryDelay();

  const std::string host_;
  const std::uint16_t port_;
  const std::size_t recv_buffer_bytes_;

  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;

  mutable std::mutex state_mutex_;
  std::optional<ReceivedPacket> latest_;
  std::deque<std::int64_t> recv_timestamps_;
  std::uint64_t sequence_ = 0;

  // Touched only by the receiver thread.
  std::uint64_t bad_packet_count_ = 0;
};

}  // namespace quest_vr
