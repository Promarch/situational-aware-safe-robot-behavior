#include "overlay_udp.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {
constexpr uint32_t kMagic = 0x4C564F52;  // "ROVL"
constexpr uint16_t kVersion = 1;
constexpr size_t kMaxPoints = 255;

class Writer {
 public:
  template <typename T>
  void put(T v) {
    const auto* b = reinterpret_cast<const uint8_t*>(&v);  // the host is little endian (x86)
    buf.insert(buf.end(), b, b + sizeof(T));
  }
  void put3(const Eigen::Vector3d& p) {
    for (int i = 0; i < 3; ++i) put(static_cast<float>(p[i]));
  }
  std::vector<uint8_t> buf;
};
}  // namespace

OverlaySender::OverlaySender(const std::string& host, uint16_t port) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) throw std::runtime_error("overlay: cannot create a UDP socket");
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
      ::connect(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd_);
    throw std::runtime_error("overlay: bad destination " + host);
  }
}

OverlaySender::~OverlaySender() {
  if (fd_ >= 0) ::close(fd_);
}

void OverlaySender::send(const supervision::Overlay& o) {
  const size_t n_points = std::min(o.predicted.size(), kMaxPoints);
  const int n_hands = std::clamp(o.hands.n_hands, 0, supervision::kMaxHands);
  Writer w;
  w.put(kMagic);
  w.put(kVersion);
  w.put(static_cast<uint8_t>(o.valid | o.moving << 1 | o.holding_object << 2 | o.slowed << 3 |
                             o.hands.unlocated << 4));
  w.put(static_cast<uint8_t>(o.contact));
  w.put(++seq_);
  w.put(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count()));
  w.put(static_cast<float>(o.cap_fraction));
  w.put3(o.reference);
  w.put(static_cast<uint8_t>(n_points));
  w.put(static_cast<uint8_t>(n_hands));
  w.put(static_cast<uint16_t>(0));
  for (size_t k = 0; k < n_points; ++k) w.put3(o.predicted[k]);
  for (int h = 0; h < n_hands; ++h) {
    w.put3(o.hands.hands[h].center);
    w.put(static_cast<float>(o.hands.hands[h].radius));
    w.put(static_cast<uint8_t>(o.status[h]));
    for (int i = 0; i < 3; ++i) w.put(static_cast<uint8_t>(0));
  }
  // Errors (e.g. ECONNREFUSED while no detector listens) are ignored: display only.
  (void)::send(fd_, w.buf.data(), w.buf.size(), MSG_DONTWAIT);
}
