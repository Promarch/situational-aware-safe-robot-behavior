// Nanoleaf test without the robot: switches the controller into streaming mode, then shows every
// status colour of pick_place (status_light.h) for 2 s, resent at 20 Hz, and ends on white.
//
// Usage: test_nanoleaf [<ip> <token> <panelId> [<panelId> ...]]
//   Without arguments: the controller and panels in nanoleaf_config.h.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "nanoleaf.h"
#include "nanoleaf_config.h"
#include "status_light.h"

int main(int argc, char** argv) {
  std::string ip = nanoleaf_config::kIp;
  std::string token = nanoleaf_config::kToken;
  std::vector<uint16_t> panels = nanoleaf_config::kPanels;
  if (argc >= 4) {
    ip = argv[1];
    token = argv[2];
    panels.clear();
    for (int i = 3; i < argc; ++i) panels.push_back(static_cast<uint16_t>(std::atoi(argv[i])));
  } else if (argc != 1) {
    std::fprintf(stderr, "usage: %s [<ip> <token> <panelId> [<panelId> ...]]\n", argv[0]);
    return 1;
  }

  try {
    const Nanoleaf leaf(ip, token, panels);
    std::string error;
    if (!leaf.activateStreaming(&error)) {
      std::fprintf(stderr, "could not activate streaming mode on %s: %s\n", ip.c_str(),
                   error.c_str());
      return 1;
    }
    std::printf("streaming to %s:%u, %zu panels\n", ip.c_str(), Nanoleaf::kStreamPort,
                panels.size());

    for (LightState s : {LightState::kNoHand, LightState::kHandClear, LightState::kIntruding,
                         LightState::kConstrained, LightState::kUnknown, LightState::kError,
                         LightState::kIdle}) {
      std::printf("%s\n", name(s));
      std::fflush(stdout);
      for (int i = 0; i < 40; ++i) {
        leaf.setAll(colour(s));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  return 0;
}
