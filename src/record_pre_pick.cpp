// Records the joint configuration of the corrected pre-pick pose (poses::kPrePick: 20 cm above
// the table, tool axis exactly vertical, fingers aligned with the base axes).
//
//   current --S-curve--> recorded pre-pick joints --Cartesian pose--> kPrePick
//   (print joints for poses.h) --S-curve--> home
//
// The robot solves IK itself through the Cartesian pose interface.
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "confirm.h"
#include "examples_common.h"
#include "motion.h"
#include "poses.h"
#include "s_curve.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";

const scurve::JointLimits kJointLimits = scurve::frankaLimits(0.3, 0.3, 0.005);
constexpr motion::CartesianLimits kTranslationLimits{0.05, 0.3, 3.0};  // m/s, m/s^2, m/s^3
constexpr motion::CartesianLimits kRotationLimits{0.2, 1.0, 10.0};     // rad/s, ...
}  // namespace

int main() {
  try {
    franka::Robot robot(kRobotIp);

    if (!confirm("S-curve to the recorded pre-pick joints")) return 0;
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    motion::moveJointSCurve(robot, poses::kPrePickJoints, kJointLimits);

    if (!confirm("Cartesian move to the corrected pre-pick pose")) return 0;
    const double duration =
        motion::moveCartesianPoseTo(robot, poses::kPrePick, kTranslationLimits, kRotationLimits);

    const franka::RobotState s = robot.readOnce();
    const std::array<double, 16>& T = s.O_T_EE;
    const double pos_error = std::sqrt(std::pow(T[12] - poses::kPrePick[12], 2) +
                                       std::pow(T[13] - poses::kPrePick[13], 2) +
                                       std::pow(T[14] - poses::kPrePick[14], 2));
    const double tilt = std::acos(std::min(1.0, -T[10]));  // angle of EE z from straight down
    std::cout << std::fixed << std::setprecision(6) << "Reached in " << duration << " s\n"
              << "Measured EE: x=" << T[12] << " y=" << T[13] << " z=" << T[14]
              << ", position error " << pos_error * 1e3 << " mm, tool tilt "
              << tilt * 180 / M_PI << " deg\n\n"
              << "constexpr JointPose kPrePickJoints = {\n    {";
    for (size_t i = 0; i < 7; ++i) std::cout << s.q[i] << (i < 6 ? ", " : "}};\n");

    if (!confirm("S-curve back to home")) return 0;
    motion::moveJointSCurve(robot, poses::kHome, kJointLimits);
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
