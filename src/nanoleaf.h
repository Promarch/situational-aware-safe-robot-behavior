// Nanoleaf light panels over the external-control (extControl v2) stream.
//
// activateStreaming() switches the controller into streaming mode once (HTTP PUT to
// http://<ip>:16021/api/v1/<token>/effects). After that every frame is a single UDP datagram to
// <ip>:60222, big endian:
//   uint16 n_panels, then n_panels x { uint16 panel id, uint8 R, G, B, W (0),
//                                      uint16 transition time [x 100 ms] }
// The panels keep the last colour until the next frame or another effect is chosen.
// Sending never blocks and errors are ignored (display only): send from any thread, including
// two at once.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

struct Rgb {
  uint8_t r = 0, g = 0, b = 0;
};

class Nanoleaf {
 public:
  static constexpr uint16_t kStreamPort = 60222;

  // Creates the UDP socket; throws std::runtime_error for an invalid IPv4 address.
  Nanoleaf(std::string ip, std::string token, std::vector<uint16_t> panels,
           uint16_t stream_port = kStreamPort);
  ~Nanoleaf();
  Nanoleaf(const Nanoleaf&) = delete;
  Nanoleaf& operator=(const Nanoleaf&) = delete;

  // Switches the controller into streaming mode (blocking HTTP request, at most `timeout_s`).
  // Returns false and fills `error` if it is unreachable or rejects the token.
  bool activateStreaming(std::string* error = nullptr, long timeout_s = 2) const;

  // Every panel the same colour.
  void setAll(Rgb colour, uint16_t transition_100ms = 1) const;
  // Per panel (ids need not be in panels()).
  void setPanels(const std::vector<std::pair<uint16_t, Rgb>>& colours,
                 uint16_t transition_100ms = 1) const;

  const std::vector<uint16_t>& panels() const { return panels_; }
  const std::string& ip() const { return ip_; }

 private:
  void send(const std::vector<uint8_t>& frame) const;

  std::string ip_;
  std::string token_;
  std::vector<uint16_t> panels_;
  int fd_ = -1;
};
