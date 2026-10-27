// Robot test of the 7-axis S-curve (joint velocity interface).
//
//   1. Reset: libfranka's example MotionGenerator moves to the recorded pre-pick joints.
//   2. Test:  our S-curve moves from pre-pick to home through the joint velocity interface.
//
// Both motions are checked with forward kinematics before anything moves (MotionGenerator is
// simulated offline), and each motion asks for its own confirmation.
//
// Usage: robot_test_s_curve [--dry-run]   (--dry-run: plan and check only, never move)
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "poses.h"
#include "s_curve.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";

// Reset speed (libfranka MotionGenerator speed factor).
constexpr double kResetSpeedFactor = 0.2;

// S-curve limits as fractions of libfranka's hardware limits.
constexpr double kFractionV = 0.3;
constexpr double kFractionA = 0.3;
constexpr double kFractionJ = 0.005;

// Minimum height above the table along any planned motion [m].
constexpr double kMinClearanceEE = 0.05;
constexpr double kMinClearanceArm = 0.10;  // flange and joint 4-7 frame origins

constexpr double kDt = 1e-3;
constexpr int kCheckEvery = 10;  // FK check every 10 ms

// Runs libfranka's MotionGenerator offline from q_start and returns its 1 kHz positions.
std::vector<std::array<double, 7>> simulateMotionGenerator(const std::array<double, 7>& q_start,
                                                           const std::array<double, 7>& q_goal) {
  MotionGenerator generator(kResetSpeedFactor, q_goal);
  franka::RobotState fake;
  fake.q_d = q_start;
  std::vector<std::array<double, 7>> path;
  franka::JointPositions cmd = generator(fake, franka::Duration(0));
  path.push_back(cmd.q);
  while (!cmd.motion_finished) {
    cmd = generator(fake, franka::Duration(1));
    path.push_back(cmd.q);
  }
  return path;
}

std::vector<std::array<double, 7>> sampleSCurve(const scurve::JointSCurve& traj) {
  std::vector<std::array<double, 7>> path;
  for (double t = 0.0; t < traj.duration(); t += kDt) path.push_back(traj.at(t).q);
  path.push_back(traj.at(traj.duration()).q);
  return path;
}

// Forward-kinematics check of a 1 kHz joint path. Prints the result; false if too low.
bool checkPath(const char* name, const std::vector<std::array<double, 7>>& path,
               const franka::Model& model, const franka::RobotState& state) {
  const franka::Frame arm_frames[] = {franka::Frame::kJoint4, franka::Frame::kJoint5,
                                      franka::Frame::kJoint6, franka::Frame::kJoint7,
                                      franka::Frame::kFlange};
  double min_z_ee = 1e9, min_z_arm = 1e9, max_speed_ee = 0.0;
  std::array<double, 16> prev_ee{};
  for (size_t k = 0; k < path.size(); k += kCheckEvery) {
    const std::array<double, 16> ee =
        model.pose(franka::Frame::kEndEffector, path[k], state.F_T_EE, state.EE_T_K);
    min_z_ee = std::min(min_z_ee, ee[14]);
    if (k > 0) {
      const double dist = std::sqrt(std::pow(ee[12] - prev_ee[12], 2) +
                                    std::pow(ee[13] - prev_ee[13], 2) +
                                    std::pow(ee[14] - prev_ee[14], 2));
      max_speed_ee = std::max(max_speed_ee, dist / (kCheckEvery * kDt));
    }
    prev_ee = ee;
    for (franka::Frame f : arm_frames) {
      min_z_arm = std::min(min_z_arm, model.pose(f, path[k], state.F_T_EE, state.EE_T_K)[14]);
    }
  }
  const bool ok = min_z_ee - poses::kTableZ >= kMinClearanceEE &&
                  min_z_arm - poses::kTableZ >= kMinClearanceArm;
  std::cout << "[" << name << "] duration " << (path.size() - 1) * kDt << " s, lowest above table: EE "
            << (min_z_ee - poses::kTableZ) * 100 << " cm, flange/joints 4-7 "
            << (min_z_arm - poses::kTableZ) * 100 << " cm, peak EE speed " << max_speed_ee
            << " m/s -> " << (ok ? "OK" : "TOO LOW") << "\n";
  return ok;
}

bool confirm(const char* what) {
  std::cout << "\nNEXT: " << what << ". Keep the user stop at hand.\n"
            << "Press Enter to continue (anything else aborts)..." << std::endl;
  std::string line;
  if (!std::getline(std::cin, line) || !line.empty()) {
    std::cout << "Aborted.\n";
    return false;
  }
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  const bool dry_run = argc > 1 && std::strcmp(argv[1], "--dry-run") == 0;
  try {
    franka::Robot robot(kRobotIp);
    franka::Model model = robot.loadModel();
    const franka::RobotState start = robot.readOnce();
    std::cout << "Start EE position: x=" << start.O_T_EE[12] << " y=" << start.O_T_EE[13]
              << " z=" << start.O_T_EE[14] << "\n";

    const scurve::JointLimits limits = scurve::frankaLimits(kFractionV, kFractionA, kFractionJ);
    const scurve::JointSCurve plan(poses::kPrePickJoints, poses::kHome, limits);
    std::cout << "S-curve limiting joint v/a/j: " << plan.limitingJointV() + 1 << "/"
              << plan.limitingJointA() + 1 << "/" << plan.limitingJointJ() + 1 << "\n";

    const bool reset_ok = checkPath("reset: MotionGenerator -> pre-pick",
                                    simulateMotionGenerator(start.q, poses::kPrePickJoints),
                                    model, start);
    const bool test_ok = checkPath("test: S-curve pre-pick -> home", sampleSCurve(plan), model, start);
    if (!reset_ok || !test_ok) {
      std::cerr << "ABORT: a planned motion comes too close to the table.\n";
      return -1;
    }
    if (dry_run) {
      std::cout << "Dry run: checks passed, not moving.\n";
      return 0;
    }

    // --- 1. Reset to pre-pick with libfranka's MotionGenerator.
    if (!confirm("reset to pre-pick with libfranka MotionGenerator")) return 0;
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    MotionGenerator to_pre_pick(kResetSpeedFactor, poses::kPrePickJoints);
    robot.control(to_pre_pick);
    std::cout << "At pre-pick.\n";

    // --- 2. S-curve to home via joint velocities.
    if (!confirm("S-curve pre-pick -> home (joint velocity interface)")) return 0;
    // The robot integrates our dq into q_d. Re-plan from q_d at the first tick so the
    // planned path starts exactly where the robot is holding.
    std::optional<scurve::JointSCurve> traj;
    double time = 0.0;
    double max_drift = 0.0;  // max |planned q - q_d| over the motion [rad]
    franka::RobotState last_state;
    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      if (!traj) {
        traj.emplace(state.q_d, poses::kHome, limits);
      }
      const scurve::JointState ref = traj->at(time);
      for (size_t i = 0; i < 7; ++i) {
        max_drift = std::max(max_drift, std::abs(ref.q[i] - state.q_d[i]));
      }
      last_state = state;
      franka::JointVelocities cmd(ref.dq);
      if (time >= traj->duration()) {
        return franka::MotionFinished(cmd);
      }
      return cmd;
    });

    double end_error = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      end_error = std::max(end_error, std::abs(last_state.q_d[i] - poses::kHome[i]));
    }
    std::cout << "Reached home in " << time << " s.\n"
              << "Max drift of q_d from plan: " << max_drift * 1e3 << " mrad\n"
              << "Final |q_d - home|: " << end_error * 1e3 << " mrad" << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
