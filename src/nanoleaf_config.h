// The Nanoleaf next to the cell (status light of pick_place, see status_light.h). The token comes
// from the controller's pairing (POST /api/v1/new while holding the power button); the panel ids
// from GET /api/v1/<token>/panelLayout/layout.
#pragma once

#include <cstdint>
#include <vector>

namespace nanoleaf_config {
constexpr const char* kIp = "192.168.2.1";
constexpr const char* kToken = "zLnFgM2B5Q8xkfMDFepKcaXYAdk5pWI4";
inline const std::vector<uint16_t> kPanels = {40609, 11120, 56397};
}  // namespace nanoleaf_config
