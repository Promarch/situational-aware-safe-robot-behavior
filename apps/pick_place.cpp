// Supervised pick and place: a screwdriver from the pick point into a pen holder.
//
//   [Enter] (home gripper, return to start) -> pre-pick -> down, grasp -> up -> pre-place
//   (tool turned 90 deg) -> down, release -> up -> start
// One confirmation at the start, then the whole task runs. Hands come over UDP (hand_udp
// protocol) from the detector or scripts/replay_hands.py; HandReceiver's thread runs the
// supervisor on every packet next to libfranka's 1 kHz control loop. While the screwdriver is
// held, the supervisor's reference point is its tip (kObjectTip in the EE frame), else the EE.
// The screwdriver lies flat and is grasped from above, so it is held along the EE x axis; at the
// pre-place pose the gripper is turned so that it hangs straight down into the holder.
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
//     dq1..dq7 measured, dq_d1..dq_d7 commanded [rad/s], tau_ext1..tau_ext7 estimated external
//     joint torques [Nm]
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

// Screwdriver.
constexpr double kGraspWidth = 0.017;        // finger distance on the handle [m]
constexpr double kGraspEpsilon = 0.01;       // accepted deviation from kGraspWidth [m]
const Eigen::Vector3d kObjectTip(0.15, 0.0, 0.0);  // tip in the EE frame [m]
constexpr double kReleaseClearance = 0.02;   // open this much wider than the handle [m]
constexpr double kGripperSpeed = 0.05;       // [m/s]
constexpr double kGraspForce = 20.0;         // [N]
constexpr double kMinTipClearance = 0.01;    // tip above the table when released, at least [m]

// Collision detection, set with setCollisionBehavior before the task (libfranka's example
// defaults are 20 Nm / 20 N). Above an "upper" threshold the robot stops with a reflex; the
// "lower" ones only raise the contact flags. As sensitive as normal motion allows: after a run
// the program prints the largest estimated external torque per joint and suggests thresholds.
constexpr std::array<double, 7> kUpperTorque = {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}};  // [Nm]
constexpr std::array<double, 7> kLowerTorque = {{5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0}};   // [Nm]
constexpr std::array<double, 6> kUpperForce = {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};   // [N, Nm]
constexpr std::array<double, 6> kLowerForce = {{5.0, 5.0, 5.0, 5.0, 5.0, 5.0}};         // [N, Nm]
constexpr double kSuggestedMargin = 1.5;  // suggested threshold = largest torque seen * margin

// Impedances of the robot's internal controllers (as libfranka's examples use).
constexpr std::array<double, 7> kJointImpedance = {{3000, 3000, 3000, 2500, 2500, 2000, 2000}};
constexpr std::array<double, 6> kCartesianImpedance = {{3000, 3000, 3000, 300, 300, 300}};

// Segments in the order they run; supervision segment id = index + 1.
const char* const kSegmentNames[] = {"start->pre-pick", "down to pick", "up from pick",
                                     "pre-pick->pre-place", "down to place", "up from place",
                                     "pre-place->start"};
struct Frame {
  double t;  // TickLog clock [s]
  supervision::HandObservation hands;
};
}  // namespace

int main(int argc, char** argv) {
  supervision::Params params;
  if (argc > 1) params.s_lim = std::atof(argv[1]);
  const auto port = static_cast<uint16_t>(argc > 2 ? std::atoi(argv[2]) : 5005);
  params.object_tip = kObjectTip;

  if (std::isnan(poses::kPrePlaceJoints[0])) {
    std::cerr << "Not configured yet: poses::kPrePlaceJoints, record it with\n  record_pose "
                 "kPrePlaceJoints --x-down --z "
              << poses::kTableZ + poses::kPrePlaceHeight << "\n";
    return 1;
  }

  try {
    franka::Robot robot(kRobotIp);
    franka::Gripper gripper(kRobotIp);
    const franka::Model model = robot.loadModel();
    const franka::RobotState start = robot.readOnce();
    const double z_grasp = poses::kTableZ + poses::kGraspHeight;

    // Segment ids 1..7 are the supervised motions of the task, in kSegmentNames order.
    supervision::Shared shared;
    supervision::Supervisor supervisor(model, start, params, &shared);

    // Where the tip will be, from the model at the recorded pre-place joints.
    auto pointAt = [&](const scurve::Vec7& q, bool tip) {
      const std::array<double, 16> T = model.pose(
          franka::Frame::kEndEffector, q, supervisor.flangeToReference(tip), start.EE_T_K);
      return Eigen::Vector3d(T[12], T[13], T[14]);
    };
    const Eigen::Vector3d ee_pre_pick = pointAt(poses::kPrePickJoints, false);
    const Eigen::Vector3d tip_pre_pick = pointAt(poses::kPrePickJoints, true);
    const Eigen::Vector3d ee_pre_place = pointAt(poses::kPrePlaceJoints, false);
    const Eigen::Vector3d tip_pre_place = pointAt(poses::kPrePlaceJoints, true);
    const double tip_released = tip_pre_place.z() - poses::kPlaceDescent - poses::kTableZ;

    std::cout << std::fixed << std::setprecision(3) << "S_lim " << params.s_lim
              << " J, intrusion radius: hand sphere + " << params.radius_margin * 100
              << " cm, horizon " << params.horizon << " s, hold after slowdown "
              << params.hold_after_slowdown << " s\n"
              << "pick: EE " << ee_pre_pick.transpose() << ", tip once held "
              << tip_pre_pick.transpose() << " -> grasp at EE height " << z_grasp * 100
              << " cm, width "
              << kGraspWidth * 100 << " cm, force " << kGraspForce << " N\n"
              << "place: EE " << ee_pre_place.transpose() << ", tip " << tip_pre_place.transpose()
              << " -> descend " << poses::kPlaceDescent * 100 << " cm, release with the tip "
              << tip_released * 100 << " cm above the table\n";
    if (tip_released < kMinTipClearance) {
      std::cerr << "Refusing: the tip would end " << tip_released * 100
                << " cm above the table. Check kPrePlaceJoints and the place heights.\n";
      return 1;
    }

    if (!confirm("pick and place: home the gripper if needed, return to start, run the whole task")) {
      return 0;
    }
    // Rejected while the user stop is pressed, so only set right before moving. The same
    // thresholds apply during acceleration and at constant speed.
    robot.setCollisionBehavior(kLowerTorque, kUpperTorque, kLowerTorque, kUpperTorque,
                               kLowerForce, kUpperForce, kLowerForce, kUpperForce);
    robot.setJointImpedance(kJointImpedance);
    robot.setCartesianImpedance(kCartesianImpedance);
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

    // --- Supervised task.
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

      const double z_place = z_pre_place - poses::kPlaceDescent;
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
    for (const char* name : {"dq", "dq_d", "tau_ext"}) {
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
      const bool at_tip = r.holding_object;  // the point the supervisor used at this tick
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
      for (const scurve::Vec7* v7 : {&r.dq, &r.dq_d, &r.tau_ext}) {
        for (double x : *v7) csv << "," << x;
      }
      csv << "\n";
    }
    // Collision thresholds: what normal motion needed.
    std::array<double, 7> tau_max{};
    for (size_t k = 0; k < log.size; ++k) {
      for (size_t i = 0; i < 7; ++i) {
        tau_max[i] = std::max(tau_max[i], std::abs(log.ticks[k].tau_ext[i]));
      }
    }
    std::cout << "\nLargest estimated external torque during the task [Nm] (reflex above):\n";
    for (size_t i = 0; i < 7; ++i) {
      std::cout << "  joint " << i + 1 << ": " << std::setprecision(2) << tau_max[i] << " ("
                << kUpperTorque[i] << ")  suggested threshold "
                << std::ceil(tau_max[i] * kSuggestedMargin * 2.0) / 2.0 << "\n";
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
