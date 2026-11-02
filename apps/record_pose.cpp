// Records a joint configuration with an axis-aligned tool orientation.
//
// Guide the robot (pilot buttons) roughly to the pose, then run this. It snaps the EE orientation
// to the nearest axis-aligned one of the chosen kind, optionally sets the EE height, moves there
// through the Cartesian pose interface (the robot solves IK) after one confirmation, and prints
// the joints for poses.h.
//   default    tool z axis straight down (grasping from above); the rotation about it snaps to
//              the nearest multiple of 90 deg, so the fingers align with the base axes
//   --x-down   EE x axis straight down (a part held along x, like the screwdriver, hangs
//              vertically); the tool z axis becomes horizontal and snaps to the nearest base axis
//   --z H      move the EE (fingertip center) to height H in the base frame [m]; else keep it
//   --free-yaw keep the current rotation about the vertical instead of snapping it to a base
//              axis (for an object that does not lie along a base axis)
//
// Usage: record_pose [NAME, default kRecordedJoints] [--x-down] [--z H] [--free-yaw]
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <franka/exception.h>
#include <franka/robot.h>

#include "confirm.h"
#include "examples_common.h"
#include "motion.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";
constexpr motion::CartesianLimits kTranslationLimits{0.05, 0.3, 3.0};  // m/s, m/s^2, m/s^3
constexpr motion::CartesianLimits kRotationLimits{0.2, 1.0, 10.0};     // rad/s, ...
// Joint 7 limits of the Panda [rad], to warn when the recorded pose leaves little room to turn.
constexpr double kQ7Limit = 2.8973;

double deg(double rad) { return rad * 180.0 / M_PI; }

// v's horizontal direction, snapped to the closest base axis (+-x or +-y) unless free_yaw.
Eigen::Vector3d horizontalDirection(const Eigen::Vector3d& v, bool free_yaw) {
  double angle = std::atan2(v.y(), v.x());
  if (!free_yaw) angle = std::round(angle / M_PI_2) * M_PI_2;
  return {std::cos(angle), std::sin(angle), 0.0};
}
}  // namespace

int main(int argc, char** argv) {
  std::string name = "kRecordedJoints";
  bool x_down = false;
  bool free_yaw = false;
  double z_target = NAN;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--x-down") {
      x_down = true;
    } else if (arg == "--free-yaw") {
      free_yaw = true;
    } else if (arg == "--z" && i + 1 < argc) {
      z_target = std::atof(argv[++i]);
    } else {
      name = arg;
    }
  }

  try {
    franka::Robot robot(kRobotIp);
    const franka::RobotState state = robot.readOnce();
    const Eigen::Affine3d T(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d down(0, 0, -1);

    // The snapped orientation: one axis straight down, one horizontal (along the nearest base
    // axis unless --free-yaw), the third completing a right-handed frame.
    Eigen::Matrix3d R_target;
    if (x_down) {
      const Eigen::Vector3d z = horizontalDirection(R.col(2), free_yaw);
      R_target << down, z.cross(down), z;
    } else {
      const Eigen::Vector3d x = horizontalDirection(R.col(0), free_yaw);
      R_target << x, down.cross(x), down;
    }
    Eigen::Affine3d target = Eigen::Affine3d::Identity();
    target.linear() = R_target;
    target.translation() = T.translation();
    if (!std::isnan(z_target)) target.translation().z() = z_target;

    const Eigen::AngleAxisd change(R.transpose() * R_target);
    std::cout << std::fixed << std::setprecision(4) << "EE now     " << T.translation().transpose()
              << "\nEE target  " << target.translation().transpose() << "  ("
              << (x_down ? "EE x straight down" : "tool z straight down") << ")\n"
              << "target axes in the base frame: x " << R_target.col(0).transpose() << ", y "
              << R_target.col(1).transpose() << ", z " << R_target.col(2).transpose() << "\n"
              << "rotation to apply: " << deg(change.angle()) << " deg\n";

    if (!confirm("move to the snapped pose (Cartesian pose)")) return 0;
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    std::array<double, 16> pose;
    Eigen::Matrix4d::Map(pose.data()) = target.matrix();
    motion::moveCartesianPoseTo(robot, pose, kTranslationLimits, kRotationLimits);

    const franka::RobotState s = robot.readOnce();
    const Eigen::Affine3d reached(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    const Eigen::AngleAxisd error(reached.linear().transpose() * R_target);
    std::cout << "reached " << reached.translation().transpose() << ", orientation error "
              << deg(error.angle()) << " deg\n\n"
              << std::setprecision(6) << "constexpr JointPose " << name << " = {\n    {";
    for (size_t i = 0; i < 7; ++i) std::cout << s.q[i] << (i < 6 ? ", " : "}};\n");
    if (std::abs(s.q[6]) > kQ7Limit - 0.3) {
      std::cout << "WARNING: joint 7 is within 0.3 rad of its limit\n";
    }
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
