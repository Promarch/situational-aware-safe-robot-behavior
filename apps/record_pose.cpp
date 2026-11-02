// Records a joint configuration with the tool pointing straight down.
//
// Guide the robot (pilot buttons) roughly to the pose, then run this. It keeps the EE position,
// makes the tool axis exactly vertical and snaps the rotation about it to the nearest multiple of
// 90 deg (fingers aligned with the base axes), moves there through the Cartesian pose interface
// (the robot solves IK) after one confirmation, and prints the joints for poses.h.
//
// Usage: record_pose [NAME, default kRecordedJoints]
#include <cmath>
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

// Rotation of the EE x axis about the base z axis [rad].
double yawOf(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }
}  // namespace

int main(int argc, char** argv) {
  const std::string name = argc > 1 ? argv[1] : "kRecordedJoints";
  try {
    franka::Robot robot(kRobotIp);
    const franka::RobotState state = robot.readOnce();
    const Eigen::Affine3d T(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    const Eigen::Matrix3d R = T.linear();

    // Target: same position, tool z straight down, yaw snapped to a multiple of 90 deg.
    const double yaw = yawOf(R);
    const double yaw_snapped = std::round(yaw / M_PI_2) * M_PI_2;
    const Eigen::Vector3d z(0, 0, -1);
    const Eigen::Vector3d x(std::cos(yaw_snapped), std::sin(yaw_snapped), 0);
    Eigen::Affine3d target = Eigen::Affine3d::Identity();
    target.linear().col(0) = x;
    target.linear().col(1) = z.cross(x);
    target.linear().col(2) = z;
    target.translation() = T.translation();

    // The pick pose (poses::kPrePickJoints) has its EE x axis along base -y: yaw -90 deg.
    const double tilt = std::acos(std::min(1.0, -R(2, 2)));
    std::cout << std::fixed << std::setprecision(4) << "EE position " << T.translation().transpose()
              << "\ntool axis " << deg(tilt) << " deg from vertical, yaw " << deg(yaw)
              << " deg -> snapped to " << deg(yaw_snapped) << " deg ("
              << deg(std::remainder(yaw_snapped + M_PI_2, 2 * M_PI))
              << " deg relative to the pick orientation)\n";

    if (!confirm("rotate the tool to straight down at the same position (Cartesian pose)")) {
      return 0;
    }
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    std::array<double, 16> pose;
    Eigen::Matrix4d::Map(pose.data()) = target.matrix();
    motion::moveCartesianPoseTo(robot, pose, kTranslationLimits, kRotationLimits);

    const franka::RobotState s = robot.readOnce();
    const Eigen::Affine3d reached(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    std::cout << "reached " << reached.translation().transpose() << ", tool axis "
              << deg(std::acos(std::min(1.0, -reached.linear()(2, 2)))) << " deg from vertical\n\n"
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
