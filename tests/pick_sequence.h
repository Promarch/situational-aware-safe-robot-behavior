// Shared by the slowdown tests: parameters of the pick sequence, its nominal EE path and the
// virtual hand schedule selected on the command line.
#pragma once

#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>

#include "poses.h"
#include "s_curve.h"
#include "segments.h"
#include "virtual_hand.h"

namespace pick_sequence {

const scurve::JointLimits kJointLimits = scurve::frankaLimits(0.5, 0.5, 0.005);
constexpr motion::CartesianLimits kVerticalLimits{0.05, 0.3, 3.0};
constexpr double kZGrasp = poses::kTableZ + poses::kGraspHeight;

// Start and end of the sequence: home with joint 1 turned +90 deg (EE on the robot's left), so
// the joint segments to and from the pick point (on the right) are long.
inline scurve::Vec7 startPose() {
  scurve::Vec7 q = poses::kHome;
  q[0] += M_PI_2;
  return q;
}
const scurve::Vec7 kStart = startPose();

inline Eigen::Vector3d eeOf(const franka::Model& model, const franka::RobotState& state,
                            const scurve::Vec7& q) {
  const std::array<double, 16> T =
      model.pose(franka::Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
  return {T[12], T[13], T[14]};
}

// Nominal EE position t seconds into each segment, and nominal segment durations.
struct Nominal {
  std::function<Eigen::Vector3d(int, double)> ee;
  std::array<double, virtual_hand::kNumSegments> duration;
};

inline Nominal nominal(const franka::Model& model, const franka::RobotState& state) {
  const scurve::JointSCurve to_pre(kStart, poses::kPrePickJoints, kJointLimits);
  const Eigen::Vector3d p_pre = eeOf(model, state, poses::kPrePickJoints);
  const scurve::Profile1D vertical = scurve::planRestToRest(
      p_pre.z() - kZGrasp, kVerticalLimits.v, kVerticalLimits.a, kVerticalLimits.j);
  Nominal n;
  n.duration = {to_pre.duration(), vertical.duration(), vertical.duration(), to_pre.duration()};
  n.ee = [&model, state, to_pre, p_pre, vertical](int segment, double t) -> Eigen::Vector3d {
    switch (segment) {
      case 0: return eeOf(model, state, to_pre.at(t).q);
      case 1: return p_pre - Eigen::Vector3d(0, 0, vertical.at(t).p);
      case 2: return Eigen::Vector3d(p_pre.x(), p_pre.y(), kZGrasp + vertical.at(t).p);
      default: return eeOf(model, state, to_pre.at(to_pre.duration() - t).q);  // reversed
    }
  };
  return n;
}

// argv[first]: a schedule file, or "--random SEED" (default: --random 1). Prints the schedule
// in file format, so a random one can be saved and edited.
inline std::vector<virtual_hand::Event> schedule(int argc, char** argv, int first,
                                                 const Nominal& n) {
  std::vector<virtual_hand::Event> events;
  if (argc > first && std::strcmp(argv[first], "--random") != 0) {
    events = virtual_hand::load(argv[first]);
    std::cout << "Hand schedule from " << argv[first] << ":\n";
  } else {
    const unsigned seed = argc > first + 1 ? std::strtoul(argv[first + 1], nullptr, 10) : 1;
    events = virtual_hand::random(seed, n.ee, n.duration);
    std::cout << "Random hand schedule, seed " << seed << ":\n";
  }
  virtual_hand::print(events, std::cout);
  return events;
}

}  // namespace pick_sequence
