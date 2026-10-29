// hand_udp_receiver_example.cpp -- how the libfranka/FCI side should consume the stream.
//
// Build:  g++ -O2 -std=c++17 -pthread hand_udp_receiver_example.cpp -o hand_udp_receiver
// Run:    ./hand_udp_receiver [port=5005] [seconds=10]
//
// The important design point: the FCI control callback runs at 1 kHz with a hard
// deadline (libfranka aborts with communication_constraints_violation if you are
// late). It must NEVER call recv(), lock a mutex that someone else may hold, print,
// or allocate. So:
//   * a normal background thread does the blocking recv() and parsing,
//   * it publishes the newest Package into a shared slot,
//   * the control callback only does a try_lock + memcpy (~1.3 kB, < 1 us) and keeps
//     its previous copy if the lock happens to be busy. try_lock never blocks.
// Here a plain loop with sleep_until stands in for robot.control(...).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <iostream>

#include "udp_listener/udp_listener.hpp"

using Clock = std::chrono::steady_clock;

struct Snapshot {
  hand_udp::Package pkg;
  Clock::time_point t_recv{};      // receiver's monotonic clock -> use for the watchdog
  std::int64_t t_recv_realtime_ns = 0;  // only to compare with the sender's timestamps
  std::uint64_t n_received = 0, n_lost = 0, n_bad = 0;
};

class HandReceiver {
 public:
  HandReceiver(std::uint16_t port, const char* bind_ip) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) { perror("socket"); std::exit(1); }
    int rcvbuf = 1 << 20;
    setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    timeval tv{0, 100000};  // 100 ms recv timeout so the thread can notice stop_
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, bind_ip, &addr.sin_addr);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      perror("bind (port already used by udp_listener.py?)");
      std::exit(1);
    }
    thread_ = std::thread(&HandReceiver::run, this);
  }
  ~HandReceiver() {
    stop_ = true;
    thread_.join();
    close(fd_);
  }

  // RT-safe: never blocks. Copies the newest snapshot into `out` if there is a
  // new one and the slot is free; otherwise leaves `out` untouched.
  bool try_get(Snapshot& out) {
    if (version_.load(std::memory_order_acquire) == out_version_) return false;  // nothing new
    std::unique_lock<std::mutex> lk(mtx_, std::try_to_lock);
    if (!lk.owns_lock()) return false;  // writer busy -> use previous copy this cycle
    out = latest_;
    out_version_ = version_.load(std::memory_order_relaxed);
    return true;
  }

 private:
  void run() {
    alignas(8) unsigned char buf[2048];
    Snapshot local;
    bool have_seq = false;
    while (!stop_) {
      ssize_t n = recv(fd_, buf, sizeof(buf), 0);
      if (n < 0) continue;  // timeout (EAGAIN) or EINTR
      auto t_recv = Clock::now();
      auto t_rt = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
      hand_udp::Package p;
      if (!hand_udp::parse(buf, static_cast<std::size_t>(n), p)) {
        ++local.n_bad;  // ignore it -- do NOT treat a broken Package as "no hands"
        continue;
      }
      // Comparison with the previous packet 
      // (have_seq) ensures that a packet was previously received
      if (have_seq) { 
        std::uint32_t last = local.pkg.header.seq;
        if (!hand_udp::seq_newer(p.header.seq, last)) {
          // older or duplicate: drop, unless the jump is huge (sender restarted)
          if (static_cast<std::uint32_t>(last - p.header.seq) < 1000) continue;
        } else {
          local.n_lost += p.header.seq - last - 1;
        }
      }
      have_seq = true;
      local.pkg = p;
      local.t_recv = t_recv;
      local.t_recv_realtime_ns = t_rt;
      ++local.n_received;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_ = local;
      }
      version_.fetch_add(1, std::memory_order_release);
    }
  }

  int fd_ = -1;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::mutex mtx_;
  Snapshot latest_;
  std::atomic<std::uint64_t> version_{0};
  std::uint64_t out_version_ = 0;  // only touched by the reader
};

// Example "distance to robot": distance of every valid landmark to ONE point.
// On the robot this would be e.g. the flange position from robot_state.O_T_EE[12..14]
// (or better: distances to capsules around every link, from the joint positions).
// Points must be in the robot base frame (frame_id == kFrameRobotBase) for this to be valid.
static float min_distance_to(const hand_udp::Package& p, const Eigen::Vector3f& ref) {
  float best = INFINITY;
  for (int h = 0; h < p.n_hands; ++h)
    for (int j = 0; j < hand_udp::kNumLandmarks; ++j) {
      // pts is (kNumLandmarks x 3): row j is one landmark, so transpose it into a column vector
      Eigen::Vector3f q = p.hands[h].pts.row(j).transpose();
      // NaN = no depth for this hand. fmin() would silently ignore it, so a hand with
      // unknown position would look "far away". Treat it as touching the robot instead.
      if (!q.allFinite()) return 0.0f;
      best = std::fmin(best, (q - ref).norm());
    }
  return best;
}

int main(int argc, char** argv) {
  const auto port = static_cast<std::uint16_t>(argc > 1 ? std::atoi(argv[1]) : 5005);
  const int seconds = argc > 2 ? std::atoi(argv[2]) : 10;
  const auto kStaleAfter = std::chrono::milliseconds(100);  // ~3 frames at 30 Hz
  const Eigen::Vector3f kRef(0.0f, 0.0f, 0.0f);             // stand-in for the flange position

  HandReceiver rx(port, "127.0.0.1");
  std::printf("listening on 127.0.0.1:%u for %d s (simulated 1 kHz loop)\n", port, seconds);

  Snapshot snap;  // the control loop's own copy
  bool ever = false;
  double lat_sum_ms = 0, lat_max_ms = 0;
  int lat_n = 0, stale_cycles = 0;
  auto t_next = Clock::now();
  auto t_print = t_next + std::chrono::seconds(1);
  const auto t_end = t_next + std::chrono::seconds(seconds);

  while (Clock::now() < t_end) {
    // ------------------------------------------------ this part = control callback
    if (rx.try_get(snap)) {
      ever = true;
      double lat = (snap.t_recv_realtime_ns - snap.pkg.header.t_send_ns) / 1e6;
      lat_sum_ms += lat; lat_max_ms = std::fmax(lat_max_ms, lat); ++lat_n;
    }
    const auto now = Clock::now();
    const bool stale = !ever || (now - snap.t_recv) > kStaleAfter;
    const bool unlocated = snap.pkg.header.flags & hand_udp::kFlagHandUnlocated;
    float dist = min_distance_to(snap.pkg, kRef);
    // Fail-safe decision. A real controller would scale velocity smoothly with
    // dist (speed & separation monitoring) instead of an on/off switch.
    bool unsafe = stale || unlocated || dist < 0.30f;
    if (stale) ++stale_cycles;
    (void)unsafe;  // -> here you would ramp the commanded velocity down
    // ------------------------------------------------ end of control callback

    if (now >= t_print) {  // printing is NOT allowed in a real RT callback
      t_print += std::chrono::seconds(1);
      std::printf("rx %llu lost %llu bad %llu | transport mean %.3f max %.3f ms | hands %d "
                  "dist %.3f m | %s%s\n",
                  (unsigned long long)snap.n_received, (unsigned long long)snap.n_lost,
                  (unsigned long long)snap.n_bad, lat_n ? lat_sum_ms / lat_n : 0.0, lat_max_ms,
                  snap.pkg.n_hands, std::isinf(dist) ? -1.0 : dist,
                  stale ? "STALE " : "", unlocated ? "UNLOCATED" : "");
      std::cout << "Pts: " << snap.pkg.hands[0].pts << ". Sphere radius: " << snap.pkg.hands[0].s_radius << std::endl;
      lat_sum_ms = lat_max_ms = 0; lat_n = 0;
    }
    t_next += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(t_next);
  }
  std::printf("stale cycles: %d of ~%d\n", stale_cycles, seconds * 1000);
  return 0;
}
