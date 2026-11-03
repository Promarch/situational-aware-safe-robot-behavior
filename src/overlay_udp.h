// Sends the supervisor's view of every camera frame (supervision::Overlay) over UDP to the hand
// detector, which draws it into its recorded video. Display only: nothing reads it back, and a
// lost packet or no listener changes nothing for the robot.
//
// Packet (little endian, packed), see also scripts/robot_overlay_protocol.py:
//   uint32 magic 'ROVL' (0x4C564F52), uint16 version (1),
//   uint8 flags: bit0 valid, bit1 moving, bit2 holding_object, bit3 slowed, bit4 hand unlocated,
//   uint8 contact of the slowdown (0 free, 1 constrained, 2 unknown),
//   uint32 seq, uint64 t_send_ns (CLOCK_REALTIME), float32 cap_fraction,
//   float32[3] reference point [m, robot base frame], uint8 n_points, uint8 n_hands, uint16 0,
//   n_points x float32[3] predicted reference points (every sample_dt over the horizon),
//   n_hands x { float32[3] center, float32 radius, uint8 status (0 clear, 1 intruding,
//               2 constrained), uint8[3] 0 }
#pragma once

#include <cstdint>
#include <string>

#include "supervision.h"

class OverlaySender {
 public:
  OverlaySender(const std::string& host, uint16_t port);  // throws std::runtime_error
  ~OverlaySender();
  OverlaySender(const OverlaySender&) = delete;
  OverlaySender& operator=(const OverlaySender&) = delete;

  void send(const supervision::Overlay& overlay);

 private:
  int fd_ = -1;
  uint32_t seq_ = 0;
};
