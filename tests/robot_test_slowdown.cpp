// Robot test of the hand-aware slowdown with a virtual hand (no camera yet).
//
//   start -joint-> pre-pick, open, -vertical-> grasp height, grasp, -vertical-> pre-pick,
//   -joint-> start, release.        (start: poses::kStart)
// The supervisor runs at the camera rate (30 Hz) in its own thread next to libfranka's 1 kHz
// control loop.
//
// Virtual hand: in each of the two joint segments it appears kHandAfterTopSpeed after the
// nominal motion reaches its top speed, on the path where the robot will be kHandLead later
// (inside the prediction horizon, so the next camera frame triggers the slowdown), and stays
// kHandDuration. It only exists in software: the robot slows down but nothing is there.
//
// Usage: robot_test_slowdown [S_lim in J, default 0.05]
// Writes, then plot with  python3 ../scripts/plot_slowdown.py robot_slowdown.csv
//   robot_slowdown.csv, one row per control tick:
//     t, segment (0 start->pre-pick, 1 down, 2 up, 3 pre-pick->start), t_segment,
//     ee_speed = |J(q) dq| [m/s], v_safe along the motion direction (every tick; empty at rest),
//     hand_present, hand_distance (EE to hand, while present), dq1..dq7 measured,
//     dq_d1..dq_d7 commanded [rad/s]
//   robot_slowdown_decisions.csv: segment, t_segment, action (slow/resume), cap_percent
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "confirm.h"
#include "examples_common.h"
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

constexpr double kFramePeriod = 1.0 / 30.0;  // camera [s]
constexpr double kHandAfterTopSpeed = 0.2;   // [s]
constexpr double kHandLead = 0.4;            // [s], < horizon
constexpr double kHandDuration = 3.0;        // [s]

struct Hand {
  uint32_t segment_id;  // 1 start->pre-pick, 4 pre-pick->start
  double t_on, t_off;   // time since the segment started [s]
  Eigen::Vector3d position;
};
}  // namespace

int main(int argc, char** argv) {
  supervision::Params params;
  if (argc > 1) params.s_lim = std::atof(argv[1]);

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

    // --- Virtual hands from the nominal joint segments. A rest-to-rest profile reaches its top
    // speed at the start of its 4th phase (after jerk up, constant acceleration, jerk down).
    auto handFor = [&](uint32_t segment_id, const scurve::JointSCurve& nominal) {
      const double t_top = nominal.sigma().segment(3).t0;
      const double t_on = t_top + kHandAfterTopSpeed;
      const Hand h{segment_id, t_on, t_on + kHandDuration, eeOf(nominal.at(t_on + kHandLead).q)};
      std::cout << "Hand in segment " << segment_id << ": nominal top speed at " << t_top
                << " s, hand appears at " << t_on << " s on the path at (" << h.position.transpose()
                << ")\n";
      return h;
    };
    const Hand hands[] = {
        handFor(1, scurve::JointSCurve(poses::kStart, poses::kPrePickJoints, kJointLimits)),
        handFor(4, scurve::JointSCurve(poses::kPrePickJoints, poses::kStart, kJointLimits))};
    auto handAt = [&](uint32_t segment_id, double t_segment) {
      supervision::HandObservation obs;
      for (const Hand& h : hands) {
        if (segment_id == h.segment_id && t_segment >= h.t_on && t_segment < h.t_off) {
          obs = {true, h.position};
        }
      }
      return obs;
    };
    std::cout << "S_lim " << params.s_lim << " J, radius " << params.radius * 100
              << " cm, horizon " << params.horizon << " s, hold after slowdown "
              << params.hold_after_slowdown << " s\n";

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
    std::atomic<bool> done{false};
    std::thread camera([&] {
      auto next = std::chrono::steady_clock::now();
      while (!done) {
        const supervision::Snapshot s = shared.snapshot.load();
        supervisor.onFrame(handAt(s.segment_id, s.t));
        next += std::chrono::microseconds(static_cast<int>(kFramePeriod * 1e6));
        std::this_thread::sleep_until(next);
      }
    });

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
    try {
      completed = sequence();
    } catch (...) {
      done = true;
      camera.join();
      throw;
    }
    done = true;
    camera.join();

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
    csv << std::setprecision(7) << "t,segment,t_segment,ee_speed,v_safe,hand_present,hand_distance";
    for (const char* name : {"dq", "dq_d"}) {
      for (int i = 1; i <= 7; ++i) csv << "," << name << i;
    }
    csv << "\n";
    for (size_t k = 0; k < log.size; ++k) {
      const motion::Tick& r = log.ticks[k];
      const Jacobian J = Eigen::Map<const Jacobian>(
          model.zeroJacobian(franka::Frame::kEndEffector, r.q, start.F_T_EE, start.EE_T_K).data());
      const Eigen::Vector3d v =
          J.topRows<3>() * Eigen::Map<const Eigen::Matrix<double, 7, 1>>(r.dq.data());
      const double speed = v.norm();
      const supervision::HandObservation hand = handAt(r.segment + 1, r.t_segment);
      csv << r.t << "," << r.segment << "," << r.t_segment << "," << speed << ",";
      if (speed > 1e-4) csv << supervisor.safeSpeed(r.q, v / speed);  // no direction at rest
      csv << "," << hand.present << ",";
      if (hand.present) csv << (eeOf(r.q) - hand.position).norm();
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
