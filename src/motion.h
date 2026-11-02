// Robot motions on the S-curve profiles. With `shared`, the supervisor (supervision.h) can slow
// a motion down or stop it on its path; with `log`, every control tick is recorded.
#pragma once

#include <chrono>
#include <vector>

#include <franka/robot.h>

#include "s_curve.h"
#include "supervision.h"

namespace motion {

struct CartesianLimits {
  double v;  // [m/s]
  double a;  // [m/s^2]
  double j;  // [m/s^3]
};

struct Tick {
  double t;           // wall time since the log was created [s]
  int segment;        // supervision segment id - 1 (-1 without supervision)
  double t_segment;   // time since the segment started [s]
  scurve::Vec7 q;     // measured joint positions
  scurve::Vec7 dq;    // measured joint velocities
  scurve::Vec7 dq_d;  // commanded joint velocities
};

// Per-tick log, allocated up front so the control loop never allocates; full -> drops ticks.
struct TickLog {
  explicit TickLog(size_t capacity) : ticks(capacity), t0(std::chrono::steady_clock::now()) {}
  double now() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  void push(const Tick& tick) {
    if (size < ticks.size()) ticks[size++] = tick;
  }
  std::vector<Tick> ticks;
  size_t size = 0;
  std::chrono::steady_clock::time_point t0;
};

// Synchronized joint S-curve from the robot's current q_d to q_goal, sent as joint velocities.
// Returns the max |q_d - q_goal| at the end [rad].
double moveJointSCurve(franka::Robot& robot, const scurve::Vec7& q_goal,
                       const scurve::JointLimits& limits, supervision::Shared* shared = nullptr,
                       TickLog* log = nullptr);

// Straight vertical EE motion to absolute height z_target (base frame), sent as Cartesian
// velocities with a 1-D S-curve; x, y and orientation are commanded to stay fixed.
// Returns the measured EE height at the end [m].
double moveVerticalTo(franka::Robot& robot, double z_target, const CartesianLimits& limits,
                      supervision::Shared* shared = nullptr, TickLog* log = nullptr);

// EE motion to an absolute pose through the Cartesian pose interface (the robot solves IK):
// straight-line position and slerp orientation, both driven by one S-curve. Not supervised.
// Returns the duration [s].
double moveCartesianPoseTo(franka::Robot& robot, const std::array<double, 16>& O_T_target,
                           const CartesianLimits& translation, const CartesianLimits& rotation);

}  // namespace motion
