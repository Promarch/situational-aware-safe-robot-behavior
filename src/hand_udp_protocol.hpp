// Copied unchanged from the udp-implementation branch (libs/udp_listener/include/udp_listener/udp_listener.hpp, a5d8301). Keep in sync.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Eigen/Core"

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "hand_udp_protocol.hpp assumes a little-endian host (x86-64 / ARM64)"
#endif

namespace hand_udp {

constexpr char kMagic[4] = {'H', 'A', 'N', 'D'};
constexpr std::uint8_t kVersion = 1;
constexpr int kNumLandmarks = 1;
constexpr int kMaxHands = 6;

// header flags
constexpr std::uint8_t kFlagHandUnlocated = 1u << 0;

// Possible frames of the points
constexpr std::uint8_t kFrameCamera = 0; // Frame of the realsense camera
constexpr std::uint8_t kFrameRobotBase = 1;  // Same frame as the robot base

// Handedness of the hand (comes from mediapipe)
constexpr std::uint8_t kLeft = 0, kRight = 1, kUnknown = 255;

// Package header
struct PackageHeader {
    char magic[4];
    std::uint8_t version;
    std::uint8_t n_hands;
    std::uint8_t flags;
    std::uint8_t frame_id;
    std::uint32_t seq; // Used to identify out of order packages. This variable get +1 everytime a new package is sent
    std::uint32_t reserved; // Placeholder for future use
    std::int64_t t_capture_ns; // Time in ns when the image was taken
    std::int64_t t_send_ns; // Time in ns when the package was sent (can be used to calculate the inference time)
};

static_assert(sizeof(PackageHeader) == 32, "Package header layout drifted");

// Definition of the hand structure
struct Hand{
    std::uint8_t handedness;
    std::uint8_t reserved[3]; // There to manually add padding
    float score; // How sure mediapipe is of the handedness
    float s_radius; // Radius of the safety sphere
    Eigen::Matrix<float, kNumLandmarks, 3> pts;
};
static_assert(sizeof(Hand) == 24, "Hand struct layout drifted");

constexpr std::size_t kMaxPacketSize = sizeof(PackageHeader) + kMaxHands * sizeof(Hand);

// Definition of the entire packet
struct Package {
    PackageHeader header{};
    std::array<Hand, kMaxHands> hands{};
    int n_hands = 0; // Seems redundant, since we already have the information in the header, but this is usefull 
                        // when doing stuff like "for (int i = 0; i < pkg.n_hands; ++i)", since a uint8_t would trigger warnings
};

// Validate and copy a received package. Returns false for anything malformed
inline bool parse(const void* data, std::size_t len, Package& out) {
    if (len < sizeof(PackageHeader)) return false;
    PackageHeader h;
    // Copy the received data to "h"
    std::memcpy(&h, data, sizeof(h));
    // Check if the header corresponds to what we expect
    if (std::memcmp(h.magic, kMagic, 4) != 0) return false;
    if (h.version != kVersion) return false;
    if (h.n_hands > kMaxHands) return false;
    if (len != sizeof(PackageHeader) + h.n_hands * sizeof(Hand)) return false;
    // Write the full package
    out.header = h;
    out.n_hands = h.n_hands;
    std::memcpy(out.hands.data(), static_cast<const char*>(data) + sizeof(PackageHeader),  // The static cast is needed to be able to add the offset via + sizeof()
        h.n_hands * sizeof(Hand));
    return true;
}

// true if b is newer than a, robust to the 2^32 wrap-around
inline bool seq_newer(std::uint32_t b, std::uint32_t a) {
  return static_cast<std::int32_t>(b - a) > 0;
}

} // namespace hand_udp