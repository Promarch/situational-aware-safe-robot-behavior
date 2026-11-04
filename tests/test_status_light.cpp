// Offline test of status_light: which colour each supervisor state shows (priorities in
// status_light.h). No robot or Nanoleaf needed.
#include <iostream>
#include <string>

#include "status_light.h"

namespace {
bool ok = true;

void expect(LightState got, LightState want, const std::string& what) {
  const bool pass = got == want;
  std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] " << what << ": " << name(got);
  if (!pass) std::cout << " (expected " << name(want) << ")";
  std::cout << "\n";
  ok &= pass;
}

supervision::Overlay withHands(int n) {
  supervision::Overlay o;
  o.valid = true;
  o.moving = true;
  o.hands.n_hands = n;
  return o;
}
}  // namespace

int main() {
  using supervision::Contact;
  using supervision::HandStatus;

  expect(lightState(supervision::Overlay{}), LightState::kNoHand, "empty overlay");
  expect(lightState(withHands(0)), LightState::kNoHand, "moving, no hand");

  supervision::Overlay o = withHands(1);
  expect(lightState(o), LightState::kHandClear, "one clear hand");
  o.moving = false;
  expect(lightState(o), LightState::kHandClear, "one hand, robot at rest");

  o = withHands(2);
  o.assessment[1].status = HandStatus::kIntruding;
  expect(lightState(o), LightState::kIntruding, "second hand intruding");
  o.assessment[0].status = HandStatus::kConstrained;
  expect(lightState(o), LightState::kConstrained, "constrained beats intruding");

  o = withHands(0);
  o.slowed = true;
  o.contact = Contact::kFree;
  expect(lightState(o), LightState::kIntruding, "slowed, hand gone (resume pending)");
  o.contact = Contact::kConstrained;
  expect(lightState(o), LightState::kConstrained, "slowed for a constrained contact");

  o = withHands(1);
  o.assessment[0].status = HandStatus::kConstrained;
  o.hands.unlocated = true;
  expect(lightState(o), LightState::kUnknown, "unlocated beats constrained");
  o = supervision::Overlay{};
  o.hands.unlocated = true;
  expect(lightState(o), LightState::kUnknown, "stale stream (invalid overlay)");

  o = withHands(1);
  o.assessment[3].status = HandStatus::kConstrained;  // beyond n_hands: stale entry
  expect(lightState(o), LightState::kHandClear, "assessments beyond n_hands ignored");

  std::cout << (ok ? "all passed\n" : "FAILED\n");
  return ok ? 0 : 1;
}
