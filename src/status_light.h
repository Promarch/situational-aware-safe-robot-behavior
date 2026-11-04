// The supervisor's state on a Nanoleaf panel, in the colours of the detector's video overlay.
// One colour for all panels; the highest-priority state wins:
//
//   red      kError        libfranka (or the program) reported an error; stays until exit
//   blue     kUnknown      a hand is seen without a position, or the camera stream is stale
//   magenta  kConstrained  a hand is exposed near the table during a downward motion
//   orange   kIntruding    a hand is exposed to the robot, or the motion is slowed down
//   yellow   kHandClear    a hand is seen, not exposed
//   green    kNoHand       no hand
//   white    kIdle         supervision not running (before and after the task)
//
// onOverlay is called from the supervisor's thread for every camera frame (Supervisor's overlay
// sink), after the speed decision; showError/showIdle from the main thread. Sending is one
// non-blocking UDP datagram: nothing here waits, and the control loop never calls it.
#pragma once

#include <atomic>
#include <cstdint>

#include "nanoleaf.h"
#include "supervision.h"

enum class LightState : uint8_t {
  kIdle,
  kNoHand,
  kHandClear,
  kIntruding,
  kConstrained,
  kUnknown,
  kError,
};

Rgb colour(LightState state);
const char* name(LightState state);

// State shown for one frame (never kIdle or kError).
LightState lightState(const supervision::Overlay& overlay);

class StatusLight {
 public:
  explicit StatusLight(const Nanoleaf& leaf) : leaf_(leaf) {}

  // Every frame: the frame's state, or red once showError was called.
  void onOverlay(const supervision::Overlay& overlay);
  // Latches the error: red now and for every later frame.
  void showError();
  // White, unless an error is latched.
  void showIdle();

  // Last state sent.
  LightState state() const { return last_.load(); }

 private:
  void show(LightState state, int repeats = 1);

  const Nanoleaf& leaf_;
  std::atomic<bool> error_{false};
  std::atomic<LightState> last_{LightState::kIdle};
};
