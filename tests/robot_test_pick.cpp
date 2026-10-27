// Robot test of a single pick: joint S-curves in the air, vertical Cartesian moves near the table.
//
//   home --S-curve--> recorded pre-pick joints --open gripper--> vertical down to grasp height
//   --grasp 15 N--> vertical up to the pre-pick height --S-curve--> home
//
// Poses are recorded joint configurations; the vertical moves start from the EE height the
// robot reports after reaching them. Each motion asks for confirmation.
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "motion.h"
#include "poses.h"
#include "s_curve.h"
#include "confirm.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";

constexpr double kResetSpeedFactor = 0.2;  // MotionGenerator back to home if not there
constexpr double kAtHomeTolerance = 1e-3;  // [rad]

// Joint S-curve limits as fractions of libfranka's hardware limits.
const scurve::JointLimits kJointLimits = scurve::frankaLimits(0.3, 0.3, 0.005);

// Vertical Cartesian motion limits.
constexpr motion::CartesianLimits kVerticalLimits{0.05, 0.3, 3.0};

// Gripper.
constexpr double kGripperSpeed = 0.05;   // [m/s]
constexpr double kGraspForce = 15.0;     // [N]
constexpr double kGraspWidth = 0.0;      // unknown object: accept any width via epsilon_outer
constexpr double kGraspEpsilonInner = 0.005;
constexpr double kGraspEpsilonOuter = 0.1;

void printJointResult(const char* name, const motion::JointMotionResult& r) {
  std::cout << name << ": " << r.duration << " s, max lag " << r.max_lag * 1e3
            << " mrad, end error " << r.end_error * 1e3 << " mrad\n";
}

void printVerticalResult(const char* name, double z_target,
                         const motion::VerticalMotionResult& r) {
  std::cout << name << ": " << r.duration << " s, commanded z error "
            << (r.z_d_end - z_target) * 1e3 << " mm, measured z error "
            << (r.z_end - z_target) * 1e3 << " mm, max xy deviation "
            << r.max_xy_deviation * 1e3 << " mm\n";
}
}  // namespace

int main() {
  try {
    franka::Robot robot(kRobotIp);
    franka::Gripper gripper(kRobotIp);

    if (gripper.readOnce().max_width < 0.01) {
      if (!confirm("gripper homing (fingers open fully, then close)")) return 0;
      gripper.homing();
    }
    const double max_width = gripper.readOnce().max_width;
    std::cout << "Gripper max_width " << max_width << " m\n";

    const franka::RobotState start = robot.readOnce();
    double home_offset = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      home_offset = std::max(home_offset, std::abs(start.q[i] - poses::kHome[i]));
    }
    if (home_offset > kAtHomeTolerance) {
      if (!confirm("reset to home with libfranka MotionGenerator")) return 0;
      setDefaultBehavior(robot);
      MotionGenerator to_home(kResetSpeedFactor, poses::kHome);
      robot.control(to_home);
    }

    if (!confirm("S-curve home -> pre-pick")) return 0;
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    printJointResult("home -> pre-pick",
                     motion::moveJointSCurve(robot, poses::kPrePickJoints, kJointLimits));

    // Vertical moves are planned from where the robot reports the EE after the joint motion.
    const double z_pre_pick = robot.readOnce().O_T_EE_d[14];
    const double z_grasp = poses::kTableZ + poses::kGraspHeight;
    std::cout << "Pre-pick EE height " << (z_pre_pick - poses::kTableZ) * 100
              << " cm above table, descending " << (z_pre_pick - z_grasp) * 100 << " cm\n";

    std::cout << "Opening gripper...\n";
    gripper.move(max_width, kGripperSpeed);

    if (!confirm("vertical down to 2 cm above the table")) return 0;
    printVerticalResult("down", z_grasp, motion::moveVerticalTo(robot, z_grasp, kVerticalLimits));

    std::cout << "Grasping with " << kGraspForce << " N...\n";
    const bool grasped = gripper.grasp(kGraspWidth, kGripperSpeed, kGraspForce,
                                       kGraspEpsilonInner, kGraspEpsilonOuter);
    const franka::GripperState after = gripper.readOnce();
    std::cout << "Grasp " << (grasped ? "succeeded" : "failed") << ", width " << after.width
              << " m, is_grasped " << std::boolalpha << after.is_grasped << "\n";

    if (!confirm("vertical up to pre-pick height")) return 0;
    printVerticalResult("up", z_pre_pick,
                        motion::moveVerticalTo(robot, z_pre_pick, kVerticalLimits));

    if (!confirm("S-curve pre-pick -> home")) return 0;
    printJointResult("pre-pick -> home", motion::moveJointSCurve(robot, poses::kHome, kJointLimits));

    if (confirm("open gripper (release)")) {
      gripper.move(max_width, kGripperSpeed);
    }
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n"
              << "If the robot is in reflex mode, recover it in Desk before the next run.\n";
    return -1;
  }
  return 0;
}
