// Connects to the robot and prints its current state once. Does not move the robot.
#include <array>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {
constexpr const char* kRobotIp = "192.168.3.8";
}

int main() {
  try {
    franka::Robot robot(kRobotIp);
    franka::RobotState state = robot.readOnce();

    std::cout << "Joint positions q [rad]:\n  ";
    for (double q : state.q) {
      std::cout << q << " ";
    }

    // O_T_EE is column-major 4x4; translation is elements 12..14.
    const std::array<double, 16>& T = state.O_T_EE;
    std::cout << "\nEE position in base frame [m]:\n  x=" << T[12] << " y=" << T[13]
              << " z=" << T[14] << "\n";

    std::cout << "Robot mode: " << static_cast<int>(state.robot_mode) << "\n";
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
