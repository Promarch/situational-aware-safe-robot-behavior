// Receives the detector's hand stream (hand_udp protocol, see hand_udp_protocol.hpp) in its own
// normal-priority thread and calls `on_frame` for every accepted packet, in the order received.
//
// Fail-safe, as the protocol requires:
//   * no valid packet for `stale_after`: the stream is dead -> on_frame gets a frame with
//     `unlocated` set (position unknown = treated as intruding), repeated every 20 ms;
//   * HAND_UNLOCATED flag or a hand with a non-finite center or radius -> `unlocated`;
//   * points not in the robot base frame (no extrinsic calibration on the sender) -> `unlocated`,
//     unless `accept_any_frame` (debugging only: positions are then in the camera frame).
// Duplicate and out-of-order packets are dropped; malformed ones are counted and ignored.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>

#include "supervision.h"

class HandReceiver {
 public:
  struct Stats {
    uint64_t received = 0;     // accepted packets
    uint64_t lost = 0;         // gaps in seq
    uint64_t bad = 0;          // malformed packets
    uint64_t wrong_frame = 0;  // packets not in the robot base frame
    uint64_t stale = 0;        // stale frames reported
    double latency_ms = 0.0;   // last receive time - sender's t_send (clocks must agree)
  };

  HandReceiver(uint16_t port, std::function<void(const supervision::HandObservation&)> on_frame,
               std::chrono::milliseconds stale_after = std::chrono::milliseconds(100),
               bool accept_any_frame = false);
  ~HandReceiver();  // stops and joins the thread

  Stats stats() const;

 private:
  void run();

  int fd_ = -1;
  std::function<void(const supervision::HandObservation&)> on_frame_;
  std::chrono::milliseconds stale_after_;
  bool accept_any_frame_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> received_{0}, lost_{0}, bad_{0}, wrong_frame_{0}, stale_{0};
  std::atomic<double> latency_ms_{0.0};
  std::thread thread_;  // last: starts after the other members exist
};
