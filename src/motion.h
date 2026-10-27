// Blocking robot motions built on the S-curve profiles.
#pragma once

#include <franka/robot.h>

#include "s_curve.h"

namespace motion {

struct JointMotionResult {
  double duration = 0.0;   // [s]
  double max_lag = 0.0;    // max |planned q - q_d| during motion, includes ~2 ms latency [rad]
  double end_error = 0.0;  // max |q_d - q_goal| at the end [rad]
};

// Synchronized joint S-curve from the robot's current q_d to q_goal, sent as joint velocities.
JointMotionResult moveJointSCurve(franka::Robot& robot, const scurve::Vec7& q_goal,
                                  const scurve::JointLimits& limits);

struct CartesianLimits {
  double v;  // [m/s]
  double a;  // [m/s^2]
  double j;  // [m/s^3]
};

struct VerticalMotionResult {
  double duration = 0.0;        // [s]
  double z_d_end = 0.0;         // commanded EE z at the end [m]
  double z_end = 0.0;           // measured EE z at the end [m]
  double max_xy_deviation = 0.0;  // max measured horizontal EE deviation from the start [m]
};

// Straight vertical EE motion to absolute height z_target (base frame), sent as Cartesian
// velocities with a 1-D S-curve; x, y and orientation are commanded to stay fixed.
VerticalMotionResult moveVerticalTo(franka::Robot& robot, double z_target,
                                    const CartesianLimits& limits);

// EE motion to an absolute pose through the Cartesian pose interface (the robot solves IK):
// straight-line position and slerp orientation, both driven by one S-curve. Returns the
// duration [s].
double moveCartesianPoseTo(franka::Robot& robot, const std::array<double, 16>& O_T_target,
                           const CartesianLimits& translation, const CartesianLimits& rotation);

}  // namespace motion
