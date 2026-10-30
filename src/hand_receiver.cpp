#include "hand_receiver.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

#include "hand_udp_protocol.hpp"

namespace {
using Clock = std::chrono::steady_clock;
constexpr int kRecvTimeoutUs = 20000;  // recv() wakes up at least every 20 ms
}  // namespace

HandReceiver::HandReceiver(uint16_t port,
                           std::function<void(const supervision::HandObservation&)> on_frame,
                           std::chrono::milliseconds stale_after, bool accept_any_frame)
    : on_frame_(std::move(on_frame)), stale_after_(stale_after), accept_any_frame_(accept_any_frame) {
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    throw std::runtime_error("hand receiver: socket(): " + std::string(std::strerror(errno)));
  }
  const int rcvbuf = 1 << 20;
  setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  const timeval tv{0, kRecvTimeoutUs};
  setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // the detector may run on another PC
  if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const std::string err = std::strerror(errno);
    close(fd_);
    throw std::runtime_error("hand receiver: cannot bind UDP port " + std::to_string(port) + ": " +
                             err + " (another listener running?)");
  }
  thread_ = std::thread(&HandReceiver::run, this);
}

HandReceiver::~HandReceiver() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  close(fd_);
}

HandReceiver::Stats HandReceiver::stats() const {
  return {received_, lost_, bad_, wrong_frame_, stale_, latency_ms_};
}

void HandReceiver::run() {
  // libfranka gives the thread calling robot.control() SCHED_FIFO at maximum priority and never
  // resets it, and new threads inherit their creator's policy. This thread must never compete
  // with the 1 kHz control loop, so make it a normal thread explicitly.
  const sched_param normal{};
  pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal);

  alignas(8) unsigned char buf[2048];
  bool have_seq = false;
  uint32_t last_seq = 0;
  Clock::time_point last_rx{};  // last valid packet
  Clock::time_point last_stale_report{};

  while (!stop_) {
    const ssize_t n = recv(fd_, buf, sizeof(buf), 0);
    const Clock::time_point now = Clock::now();
    hand_udp::Package pkg;
    if (n < 0 || !hand_udp::parse(buf, static_cast<size_t>(n), pkg)) {
      if (n >= 0) ++bad_;  // garbage must not keep a dead stream looking alive
      // Timeout or nothing valid: once the stream has been silent for stale_after, report a
      // frame with an unknown hand every recv timeout.
      if (now - last_rx > stale_after_ && now - last_stale_report >= stale_after_ / 5) {
        supervision::HandObservation stale;
        stale.unlocated = true;
        ++stale_;
        last_stale_report = now;
        on_frame_(stale);
      }
      continue;
    }

    // After a silence no reordering is possible: accept any seq as a fresh start.
    if (now - last_rx > stale_after_) have_seq = false;
    last_rx = now;
    if (have_seq) {
      if (!hand_udp::seq_newer(pkg.header.seq, last_seq)) {
        // older or duplicate: drop, unless it jumped far back (sender restarted)
        if (static_cast<uint32_t>(last_seq - pkg.header.seq) < 1000) continue;
      } else {
        lost_ += pkg.header.seq - last_seq - 1;
      }
    }
    have_seq = true;
    last_seq = pkg.header.seq;
    ++received_;
    const int64_t now_realtime_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
    latency_ms_ = (now_realtime_ns - pkg.header.t_send_ns) / 1e6;

    supervision::HandObservation obs;
    obs.unlocated = (pkg.header.flags & hand_udp::kFlagHandUnlocated) != 0;
    if (pkg.header.frame_id != hand_udp::kFrameRobotBase) ++wrong_frame_;
    if (pkg.header.frame_id != hand_udp::kFrameRobotBase && !accept_any_frame_) {
      obs.unlocated = pkg.n_hands > 0;  // positions we cannot use
    } else {
      for (int h = 0; h < pkg.n_hands && h < supervision::kMaxHands; ++h) {
        const hand_udp::Hand& hand = pkg.hands[h];
        const Eigen::Vector3d center = hand.pts.row(0).transpose().cast<double>();
        if (!center.allFinite() || !std::isfinite(hand.s_radius)) {
          obs.unlocated = true;
          continue;
        }
        obs.hands[obs.n_hands++] = {center, hand.s_radius};
      }
    }
    on_frame_(obs);
  }
}
