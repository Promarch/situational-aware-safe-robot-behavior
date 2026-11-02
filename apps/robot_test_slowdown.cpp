// Robot test of the hand-aware slowdown with hands received over UDP (hand_udp protocol).
//
//   start -joint-> pre-pick, open, -vertical-> grasp height, grasp, -vertical-> pre-pick,
//   -joint-> start, release.        (start: poses::kStart)
// HandReceiver's thread runs the supervisor on every received packet, next to libfranka's
// 1 kHz control loop. Hands come from the detector or from scripts/replay_hands.py; start the
// sender first: without a stream the receiver reports the hand position as unknown and the
// robot moves at the energy-limited speed only.
//
// Usage: robot_test_slowdown [S_lim in J, default 0.05] [UDP port, default 5005]
// Writes, then plot with  python3 ../scripts/plot_slowdown.py robot_slowdown.csv
//   robot_slowdown.csv, one row per control tick:
//     t, segment (0 start->pre-pick, 1 down, 2 up, 3 pre-pick->start), t_segment,
//     ee_speed = |J(q) dq| [m/s], v_safe along the motion direction (every tick; empty at rest),
//     hand_present (a hand seen, or position unknown), hand_unlocated (position unknown or
//     stream stale), hand_clearance (EE distance to the nearest located hand's intrusion
//     boundary, sphere radius + margin; < 0 inside), dq1..dq7 measured,
//     dq_d1..dq_d7 commanded [rad/s]
//   robot_slowdown_decisions.csv: segment, t_segment, action (slow/resume), cap_percent
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "confirm.h"
#include "examples_common.h"
#include "hand_receiver.h"
#include "motion.h"
#include "poses.h"
#include "s_curve.h"
#include "supervision.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";

const scurve::JointLimits kJointLimits = scurve::frankaLimits(0.5, 0.5, 0.005);
constexpr motion::CartesianLimits kVerticalLimits{0.05, 0.3, 3.0};
constexpr double kResetSpeedFactor = 0.2;  // MotionGenerator back to the start if not there

constexpr double kGripperSpeed = 0.05;  // [m/s]
constexpr double kGraspForce = 15.0;    // [N]
constexpr double kGraspWidth = 0.048;   // test object [m]
constexpr double kGraspEpsilon = 0.01;  // [m]

struct Frame {
  double t;  // TickLog clock [s]
  supervision::HandObservation hands;
};
}  // namespace

int main(int argc, char** argv) {
  supervision::Params params;
  if (argc > 1) params.s_lim = std::atof(argv[1]);
  const auto port = static_cast<uint16_t>(argc > 2 ? std::atoi(argv[2]) : 5005);

  try {
    franka::Robot robot(kRobotIp);
    franka::Gripper gripper(kRobotIp);
    const franka::Model model = robot.loadModel();
    const franka::RobotState start = robot.readOnce();
    auto eeOf = [&](const scurve::Vec7& q) {
      const std::array<double, 16> T =
          model.pose(franka::Frame::kEndEffector, q, start.F_T_EE, start.EE_T_K);
      return Eigen::Vector3d(T[12], T[13], T[14]);
    };

    std::cout << "S_lim " << params.s_lim << " J, intrusion radius: hand sphere + "
              << params.radius_margin * 100 << " cm, horizon " << params.horizon
              << " s, hold after slowdown " << params.hold_after_slowdown << " s\n";

    // --- Preparation.
    if (gripper.readOnce().max_width < 0.01) {
      if (!confirm("gripper homing (fingers open fully, then close)")) return 0;
      gripper.homing();
    }
    const double max_width = gripper.readOnce().max_width;

    double start_offset = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      start_offset = std::max(start_offset, std::abs(start.q[i] - poses::kStart[i]));
    }
    if (start_offset > 1e-3) {
      if (!confirm("reset to the start pose (home, joint 1 +90 deg) with MotionGenerator")) {
        return 0;
      }
      setDefaultBehavior(robot);
      MotionGenerator to_start(kResetSpeedFactor, poses::kStart);
      robot.control(to_start);
    }

    // --- Supervised sequence. Segment ids 1..4 are the four motions below, in order.
    supervision::Shared shared;
    supervision::Supervisor supervisor(model, start, params, &shared);
    motion::TickLog log(300000);  // 5 min of control ticks
    std::vector<Frame> frames;    // written by the receiver thread, read after it stopped
    frames.reserve(20000);

    auto sequence = [&]() -> bool {
      if (!confirm("S-curve start -> pre-pick (supervised)")) return false;
      // Rejected while the user stop is pressed, so only set right before moving.
      setDefaultBehavior(robot);
      double err = motion::moveJointSCurve(robot, poses::kPrePickJoints, kJointLimits, &shared,
                                           &log);
      std::cout << "start -> pre-pick done, end error " << err * 1e3 << " mrad\n";
      const double z_pre_pick = robot.readOnce().O_T_EE_d[14];
      const double z_grasp = poses::kTableZ + poses::kGraspHeight;

      gripper.move(max_width, kGripperSpeed);
      if (!confirm("vertical down to 2 cm above the table (supervised)")) return false;
      double z = motion::moveVerticalTo(robot, z_grasp, kVerticalLimits, &shared, &log);
      std::cout << "down done, z error " << (z - z_grasp) * 1e3 << " mm\n";

      const bool grasped =
          gripper.grasp(kGraspWidth, kGripperSpeed, kGraspForce, kGraspEpsilon, kGraspEpsilon);
      std::cout << "grasp " << (grasped ? "succeeded" : "failed") << ", width "
                << gripper.readOnce().width << " m\n";

      if (!confirm("vertical up to pre-pick height (supervised)")) return false;
      z = motion::moveVerticalTo(robot, z_pre_pick, kVerticalLimits, &shared, &log);
      std::cout << "up done, z error " << (z - z_pre_pick) * 1e3 << " mm\n";

      if (!confirm("S-curve pre-pick -> start (supervised)")) return false;
      err = motion::moveJointSCurve(robot, poses::kStart, kJointLimits, &shared, &log);
      std::cout << "pre-pick -> start done, end error " << err * 1e3 << " mrad\n";
      return true;
    };
    bool completed = false;
    HandReceiver::Stats rx;
    {
      // Stops and joins its thread when leaving this block, also on an exception.
      HandReceiver receiver(port, [&](const supervision::HandObservation& hands) {
        frames.push_back({log.now(), hands});
        supervisor.onFrame(hands);
      });
      std::cout << "Listening for hands on UDP port " << port << "\n";
      completed = sequence();
      rx = receiver.stats();
    }
    std::cout << "UDP: " << rx.received << " packets, " << rx.lost << " lost, " << rx.bad
              << " malformed, " << rx.wrong_frame << " not in the robot base frame, " << rx.stale
              << " stale frames, last latency " << rx.latency_ms << " ms\n";

    // --- Results.
    const char* const names[] = {"start->pre-pick", "down", "up", "pre-pick->start"};
    std::cout << "\nSupervisor decisions:\n";
    std::ofstream dec("robot_slowdown_decisions.csv");
    dec << "segment,t_segment,action,cap_percent\n";
    for (const supervision::Decision& d : supervisor.decisions()) {
      std::cout << "  " << names[d.segment_id - 1] << " t=" << d.t_segment << " s: "
                << (d.slow_down ? "slow down" : "resume") << ", cap " << d.cap_fraction * 100
                << "%\n";
      dec << d.segment_id - 1 << "," << d.t_segment << "," << (d.slow_down ? "slow" : "resume")
          << "," << d.cap_fraction * 100 << "\n";
    }

    using Jacobian = Eigen::Matrix<double, 6, 7>;
    std::ofstream csv("robot_slowdown.csv");
    csv << std::setprecision(7) << "t,segment,t_segment,ee_speed,v_safe,hand_present,hand_unlocated,hand_clearance";
    for (const char* name : {"dq", "dq_d"}) {
      for (int i = 1; i <= 7; ++i) csv << "," << name << i;
    }
    csv << "\n";
    size_t f = 0;  // latest camera frame at or before the tick
    for (size_t k = 0; k < log.size; ++k) {
      const motion::Tick& r = log.ticks[k];
      while (f + 1 < frames.size() && frames[f + 1].t <= r.t) ++f;
      const supervision::HandObservation hands_now =
          !frames.empty() && frames[f].t <= r.t ? frames[f].hands : supervision::HandObservation{};
      const Jacobian J = Eigen::Map<const Jacobian>(
          model.zeroJacobian(franka::Frame::kEndEffector, r.q, start.F_T_EE, start.EE_T_K).data());
      const Eigen::Vector3d v =
          J.topRows<3>() * Eigen::Map<const Eigen::Matrix<double, 7, 1>>(r.dq.data());
      const double speed = v.norm();
      csv << r.t << "," << r.segment << "," << r.t_segment << "," << speed << ",";
      if (speed > 1e-4) csv << supervisor.safeSpeed(r.q, v / speed);  // no direction at rest
      csv << "," << (hands_now.n_hands > 0 || hands_now.unlocated) << "," << hands_now.unlocated
          << ",";
      if (hands_now.n_hands > 0) {
        double clearance = INFINITY;
        for (int h = 0; h < hands_now.n_hands; ++h) {
          const supervision::HandSphere& hand = hands_now.hands[h];
          clearance = std::min(clearance, (eeOf(r.q) - hand.center).norm() - hand.radius -
                                              params.radius_margin);
        }
        csv << clearance;
      }
      for (const scurve::Vec7* v7 : {&r.dq, &r.dq_d}) {
        for (double x : *v7) csv << "," << x;
      }
      csv << "\n";
    }
    std::cout << "Wrote robot_slowdown.csv (" << log.size << " ticks"
              << (log.size == log.ticks.size() ? ", log full" : "")
              << ") and robot_slowdown_decisions.csv\n"
              << "Plot: python3 ../scripts/plot_slowdown.py robot_slowdown.csv\n";

    if (completed && confirm("open gripper (release)")) {
      gripper.move(max_width, kGripperSpeed);
    }
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n"
              << "If the robot is in reflex mode, recover it in Desk before the next run.\n";
    return -1;
  }
  return 0;
}
