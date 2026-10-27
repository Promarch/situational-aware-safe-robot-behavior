// Per-tick logic of one motion segment, shared by the robot control callback and the offline
// simulator. Each step() applies a pending speed cap from the supervisor (if any), publishes a
// snapshot and returns the velocity command. Nothing here allocates or blocks.
#pragma once

#include <array>

#include "s_curve.h"
#include "supervision.h"

namespace motion {

struct CartesianLimits {
  double v;  // [m/s]
  double a;  // [m/s^2]
  double j;  // [m/s^3]
};

// Joint S-curve from the first commanded position to q_goal.
class JointSegment {
 public:
  // `shared` may be null (no supervision).
  JointSegment(const scurve::Vec7& q_goal, const scurve::JointLimits& limits,
               supervision::Shared* shared);

  // t: time since the segment started; q_d: the robot's commanded joint position.
  scurve::JointState step(double t, const scurve::Vec7& q_d);
  bool finished(double t) const { return started_ && traj_.finished(t); }

 private:
  scurve::Vec7 q_goal_;
  scurve::JointLimits limits_;
  supervision::Shared* shared_;
  uint32_t id_ = 0;
  bool started_ = false;
  scurve::JointSCurve traj_;
  uint32_t applied_seq_ = 0;
  double cap_ = 0.0;
};

// Vertical EE line to absolute height z_target, as a 1-D S-curve on the travelled distance.
class VerticalSegment {
 public:
  VerticalSegment(double z_target, const CartesianLimits& limits, supervision::Shared* shared);

  // O_T_EE_d: commanded EE pose; q: current joints (for the supervisor's mass model).
  // Returns the vertical velocity command [m/s].
  double step(double t, const std::array<double, 16>& O_T_EE_d, const scurve::Vec7& q);
  bool finished(double t) const;

 private:
  double z_target_;
  CartesianLimits limits_;
  supervision::Shared* shared_;
  uint32_t id_ = 0;
  bool started_ = false;
  double distance_ = 0.0, direction_ = 1.0;
  std::array<double, 3> p_start_{};
  scurve::Profile1D profile_;
  double t_replan_ = 0.0;
  bool replanned_ = false;
  uint32_t applied_seq_ = 0;
  double cap_ = 0.0;
};

}  // namespace motion
