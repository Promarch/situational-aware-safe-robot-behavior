#include "status_light.h"

namespace {
constexpr uint16_t kTransition = 1;  // [x 100 ms]
constexpr int kErrorRepeats = 3;     // UDP may drop a datagram; the error matters most
}  // namespace

Rgb colour(LightState state) {
  switch (state) {
    case LightState::kNoHand:      return {0, 255, 0};
    case LightState::kHandClear:   return {255, 200, 0};
    case LightState::kIntruding:   return {255, 80, 0};
    case LightState::kConstrained: return {255, 0, 255};
    case LightState::kUnknown:     return {0, 0, 255};
    case LightState::kError:       return {255, 0, 0};
    case LightState::kIdle:        break;
  }
  return {255, 255, 255};
}

const char* name(LightState state) {
  switch (state) {
    case LightState::kNoHand:      return "green: no hand";
    case LightState::kHandClear:   return "yellow: hand seen, clear";
    case LightState::kIntruding:   return "orange: hand intruding, slowed";
    case LightState::kConstrained: return "magenta: constrained contact near the table";
    case LightState::kUnknown:     return "blue: hand position unknown / stream stale";
    case LightState::kError:       return "red: error";
    case LightState::kIdle:        break;
  }
  return "white: supervision not running";
}

LightState lightState(const supervision::Overlay& o) {
  using supervision::Contact;
  using supervision::HandStatus;
  if (o.hands.unlocated) return LightState::kUnknown;
  bool constrained = o.slowed && o.contact == Contact::kConstrained;
  bool intruding = o.slowed;
  for (int h = 0; h < o.hands.n_hands && h < supervision::kMaxHands; ++h) {
    constrained |= o.assessment[h].status == HandStatus::kConstrained;
    intruding |= o.assessment[h].status == HandStatus::kIntruding;
  }
  if (constrained) return LightState::kConstrained;
  if (intruding) return LightState::kIntruding;
  if (o.hands.n_hands > 0) return LightState::kHandClear;
  return LightState::kNoHand;
}

void StatusLight::onOverlay(const supervision::Overlay& overlay) {
  show(error_.load() ? LightState::kError : lightState(overlay));
}

void StatusLight::showError() {
  error_.store(true);
  show(LightState::kError, kErrorRepeats);
}

void StatusLight::showIdle() {
  if (!error_.load()) show(LightState::kIdle);
}

void StatusLight::show(LightState state, int repeats) {
  last_.store(state);
  for (int i = 0; i < repeats; ++i) leaf_.setAll(colour(state), kTransition);
}
