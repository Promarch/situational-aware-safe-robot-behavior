// Supervised pick and place: a screwdriver from the pick point into a pen holder.
//
//   [Enter] (home gripper, return to start) -> pre-pick -> down, grasp -> up -> pre-place
//   (tool turned 90 deg) -> down, release -> up -> start
// One confirmation at the start, then the whole task runs. Hands come over UDP (hand_udp
// protocol) from the detector or scripts/replay_hands.py; HandReceiver's thread runs the
// supervisor on every packet next to libfranka's 1 kHz control loop. While the screwdriver is
// held, the supervisor's reference point is its tip (kObjectTipOffset below the EE), else the EE.
// Start the sender first: without a stream the hand position counts as unknown and every motion
// runs at the energy-limited speed only.
//
// Usage: pick_place [S_lim in J, default 0.05] [UDP port, default 5005]
// Writes, then plot with  python3 ../scripts/plot_slowdown.py pick_place.csv
//   pick_place.csv, one row per control tick:
//     t, segment (0..6, see kSegmentNames), t_segment,
//     ee_speed = |J(q) dq| of the reference point [m/s], v_safe there along its motion
//     direction (every tick; empty at rest), hand_present (a hand seen, or position unknown),
//     hand_unlocated (position unknown or stream stale), hand_clearance (reference point to the
//     nearest located hand's intrusion boundary, sphere radius + margin; < 0 inside),
//     dq1..dq7 measured, dq_d1..dq_d7 commanded [rad/s]
//   pick_place_decisions.csv: segment, t_segment, action (slow/resume), cap_percent
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

// Screwdriver. NaN until measured.
constexpr double kGraspWidth = NAN;          // handle diameter where it is grasped [m]
constexpr double kGraspEpsilon = 0.01;       // accepted deviation from kGraspWidth [m]
constexpr double kObjectTipOffset = NAN;     // EE (fingertip center) to the tip, along tool z [m]
constexpr double kReleaseClearance = 0.02;   // open this much wider than the handle [m]
constexpr double kGripperSpeed = 0.05;       // [m/s]
constexpr double kGraspForce = 15.0;         // [N]

// Segments in the order they run; supervision segment id = index + 1.
const char* const kSegmentNames[] = {"start->pre-pick", "down to pick", "up from pick",
                                     "pre-pick->pre-place", "down to place", "up from place",
                                     "pre-place->start"};
// Segments that move with the screwdriver in the gripper (between grasp and release).
bool holdsObject(int segment) { return segment >= 2 && segment <= 4; }

struct Frame {
  double t;  // TickLog clock [s]
  supervision::HandObservation hands;
};
}  // namespace

int main(int argc, char** argv) {
  supervision::Params params;
  if (argc > 1) params.s_lim = std::atof(argv[1]);
  const auto port = static_cast<uint16_t>(argc > 2 ? std::atoi(argv[2]) : 5005);
  params.object_tip_offset = kObjectTipOffset;

  // Refuse to move with values that were never filled in.
  const char* missing = std::isnan(poses::kPrePlaceJoints[0]) ? "poses::kPrePlaceJoints (record_pose)"
                        : std::isnan(poses::kPlaceHeight)     ? "poses::kPlaceHeight"
                        : std::isnan(kGraspWidth)             ? "kGraspWidth"
                        : std::isnan(kObjectTipOffset)        ? "kObjectTipOffset"
                                                              : nullptr;
  if (missing != nullptr) {
    std::cerr << "Not configured yet: " << missing << "\n";
    return 1;
  }

  try {
    franka::Robot robot(kRobotIp);
    franka::Gripper gripper(kRobotIp);
    const franka::Model model = robot.loadModel();
    const franka::RobotState start = robot.readOnce();
    const double z_grasp = poses::kTableZ + poses::kGraspHeight;
    const double z_place = poses::kTableZ + poses::kPlaceHeight;

    std::cout << "S_lim " << params.s_lim << " J, intrusion radius: hand sphere + "
              << params.radius_margin * 100 << " cm, horizon " << params.horizon
              << " s, hold after slowdown " << params.hold_after_slowdown << " s\n"
              << "grasp at EE height " << z_grasp * 100 << " cm, width " << kGraspWidth * 100
              << " cm; release at EE height " << z_place * 100 << " cm; object tip "
              << kObjectTipOffset * 100 << " cm below the EE\n";

    if (!confirm("pick and place: home the gripper if needed, return to start, run the whole task")) {
      return 0;
    }
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    if (gripper.readOnce().max_width < 0.01) gripper.homing();
    const double max_width = gripper.readOnce().max_width;

    double start_offset = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      start_offset = std::max(start_offset, std::abs(start.q[i] - poses::kStart[i]));
    }
    if (start_offset > 1e-3) {
      std::cout << "returning to start\n";
      MotionGenerator to_start(kResetSpeedFactor, poses::kStart);
      robot.control(to_start);
    }

    // --- Supervised task. Segment ids 1..7 are the motions below, in kSegmentNames order.
    supervision::Shared shared;
    supervision::Supervisor supervisor(model, start, params, &shared);
    motion::TickLog log(300000);  // 5 min of control ticks
    std::vector<Frame> frames;    // written by the receiver thread, read after it stopped
    frames.reserve(20000);

    auto task = [&]() -> bool {
      double err = motion::moveJointSCurve(robot, poses::kPrePickJoints, kJointLimits, &shared,
                                           &log);
      std::cout << "start -> pre-pick, end error " << err * 1e3 << " mrad\n";
      const double z_pre_pick = robot.readOnce().O_T_EE_d[14];

      gripper.move(max_width, kGripperSpeed);
      double z = motion::moveVerticalTo(robot, z_grasp, kVerticalLimits, &shared, &log);
      std::cout << "down to pick, z error " << (z - z_grasp) * 1e3 << " mm\n";

      const bool grasped =
          gripper.grasp(kGraspWidth, kGripperSpeed, kGraspForce, kGraspEpsilon, kGraspEpsilon);
      std::cout << "grasp " << (grasped ? "succeeded" : "FAILED") << ", width "
                << gripper.readOnce().width * 100 << " cm\n";
      shared.holding_object = grasped;  // robot at rest: switch the reference point now

      z = motion::moveVerticalTo(robot, z_pre_pick, kVerticalLimits, &shared, &log);
      std::cout << "up from pick, z error " << (z - z_pre_pick) * 1e3 << " mm\n";
      if (!grasped) {
        // Still supervised; in the log this segment shows as "pre-pick->pre-place".
        std::cout << "nothing to place: back to start\n";
        motion::moveJointSCurve(robot, poses::kStart, kJointLimits, &shared, &log);
        return false;
      }

      err = motion::moveJointSCurve(robot, poses::kPrePlaceJoints, kJointLimits, &shared, &log);
      std::cout << "pre-pick -> pre-place, end error " << err * 1e3 << " mrad\n";
      const double z_pre_place = robot.readOnce().O_T_EE_d[14];

      z = motion::moveVerticalTo(robot, z_place, kVerticalLimits, &shared, &log);
      std::cout << "down to place, z error " << (z - z_place) * 1e3 << " mm\n";
      // Open only a little: the fingers are inside the holder.
      gripper.move(std::min(max_width, kGraspWidth + kReleaseClearance), kGripperSpeed);
      shared.holding_object = false;
      std::cout << "released\n";

      z = motion::moveVerticalTo(robot, z_pre_place, kVerticalLimits, &shared, &log);
      std::cout << "up from place, z error " << (z - z_pre_place) * 1e3 << " mm\n";

      err = motion::moveJointSCurve(robot, poses::kStart, kJointLimits, &shared, &log);
      std::cout << "pre-place -> start, end error " << err * 1e3 << " mrad\n";
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
      completed = task();
      rx = receiver.stats();
    }
    std::cout << (completed ? "Task completed.\n" : "Task NOT completed.\n") << "UDP: "
              << rx.received << " packets, " << rx.lost << " lost, " << rx.bad << " malformed, "
              << rx.wrong_frame << " not in the robot base frame, " << rx.stale
              << " stale frames, last latency " << rx.latency_ms << " ms\n";

    // --- Results.
    std::cout << "\nSupervisor decisions:\n";
    std::ofstream dec("pick_place_decisions.csv");
    dec << "segment,t_segment,action,cap_percent\n";
    for (const supervision::Decision& d : supervisor.decisions()) {
      std::cout << "  " << kSegmentNames[d.segment_id - 1] << " t=" << d.t_segment << " s: "
                << (d.slow_down ? "slow down" : "resume") << ", cap " << d.cap_fraction * 100
                << "%\n";
      dec << d.segment_id - 1 << "," << d.t_segment << "," << (d.slow_down ? "slow" : "resume")
          << "," << d.cap_fraction * 100 << "\n";
    }

    using Jacobian = Eigen::Matrix<double, 6, 7>;
    std::ofstream csv("pick_place.csv");
    csv << std::setprecision(7)
        << "t,segment,t_segment,ee_speed,v_safe,hand_present,hand_unlocated,hand_clearance";
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
      // Same reference point as the supervisor used in this segment.
      const bool at_tip = holdsObject(r.segment);
      const std::array<double, 16>& F_T_ref = supervisor.flangeToReference(at_tip);
      const Jacobian J = Eigen::Map<const Jacobian>(
          model.zeroJacobian(franka::Frame::kEndEffector, r.q, F_T_ref, start.EE_T_K).data());
      const Eigen::Vector3d v =
          J.topRows<3>() * Eigen::Map<const Eigen::Matrix<double, 7, 1>>(r.dq.data());
      const double speed = v.norm();
      csv << r.t << "," << r.segment << "," << r.t_segment << "," << speed << ",";
      if (speed > 1e-4) csv << supervisor.safeSpeed(r.q, v / speed, at_tip);  // none at rest
      csv << "," << (hands_now.n_hands > 0 || hands_now.unlocated) << "," << hands_now.unlocated
          << ",";
      if (hands_now.n_hands > 0) {
        const std::array<double, 16> T =
            model.pose(franka::Frame::kEndEffector, r.q, F_T_ref, start.EE_T_K);
        const Eigen::Vector3d p(T[12], T[13], T[14]);
        double clearance = INFINITY;
        for (int h = 0; h < hands_now.n_hands; ++h) {
          const supervision::HandSphere& hand = hands_now.hands[h];
          clearance = std::min(clearance,
                               (p - hand.center).norm() - hand.radius - params.radius_margin);
        }
        csv << clearance;
      }
      for (const scurve::Vec7* v7 : {&r.dq, &r.dq_d}) {
        for (double x : *v7) csv << "," << x;
      }
      csv << "\n";
    }
    std::cout << "Wrote pick_place.csv (" << log.size << " ticks"
              << (log.size == log.ticks.size() ? ", log full" : "")
              << ") and pick_place_decisions.csv\n"
              << "Plot: python3 ../scripts/plot_slowdown.py pick_place.csv\n";
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n"
              << "If the robot is in reflex mode, recover it in Desk before the next run.\n";
    return -1;
  }
  return 0;
}
