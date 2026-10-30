// UDP reception test without the robot: listens for the hand stream and prints once per second
// what arrived. Run the sender (scripts/replay_hands.py or the real detector) next to it.
//
// Usage: test_hand_receiver [port=5005] [seconds=20]
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

#include "hand_receiver.h"

int main(int argc, char** argv) {
  const auto port = static_cast<uint16_t>(argc > 1 ? std::atoi(argv[1]) : 5005);
  const int seconds = argc > 2 ? std::atoi(argv[2]) : 20;

  std::mutex mutex;
  supervision::HandObservation last;
  uint64_t frames = 0;
  HandReceiver receiver(port, [&](const supervision::HandObservation& obs) {
    std::lock_guard<std::mutex> lock(mutex);
    last = obs;
    ++frames;
  });
  std::printf("listening on UDP port %u for %d s\n", port, seconds);

  for (int s = 1; s <= seconds; ++s) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const HandReceiver::Stats st = receiver.stats();
    std::lock_guard<std::mutex> lock(mutex);
    std::printf("%3d s | frames %llu: rx %llu lost %llu bad %llu wrong-frame %llu stale %llu | "
                "latency %.2f ms | now: %s",
                s, (unsigned long long)frames, (unsigned long long)st.received,
                (unsigned long long)st.lost, (unsigned long long)st.bad,
                (unsigned long long)st.wrong_frame, (unsigned long long)st.stale, st.latency_ms,
                last.unlocated ? "UNLOCATED/STALE " : "");
    if (last.n_hands == 0 && !last.unlocated) std::printf("no hand");
    for (int h = 0; h < last.n_hands; ++h) {
      const supervision::HandSphere& hand = last.hands[h];
      std::printf("hand (%.3f, %.3f, %.3f) r=%.2f  ", hand.center.x(), hand.center.y(),
                  hand.center.z(), hand.radius);
    }
    std::printf("\n");
  }
  return 0;
}
