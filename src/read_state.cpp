// Connects to the robot and gripper and prints their current state once. Does not move anything.
#include <array>
#include <iomanip>
#include <iostream>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

namespace {
constexpr const char* kRobotIp = "192.168.3.8";

const char* robotModeName(franka::RobotMode mode) {
  switch (mode) {
    case franka::RobotMode::kOther: return "Other";
    case franka::RobotMode::kIdle: return "Idle";
    case franka::RobotMode::kMove: return "Move";
    case franka::RobotMode::kGuiding: return "Guiding";
    case franka::RobotMode::kReflex: return "Reflex";
    case franka::RobotMode::kUserStopped: return "UserStopped";
    case franka::RobotMode::kAutomaticErrorRecovery: return "AutomaticErrorRecovery";
  }
  return "Unknown";
}

// Prints a column-major 4x4 transform as rows.
void printTransform(const char* name, const std::array<double, 16>& T) {
  std::cout << name << " (rows):\n";
  for (int r = 0; r < 4; ++r) {
    std::cout << "  ";
    for (int c = 0; c < 4; ++c) {
      std::cout << std::setw(10) << T[c * 4 + r] << " ";
    }
    std::cout << "\n";
  }
}

template <size_t N>
void printArray(const std::array<double, N>& a) {
  std::cout << "{{";
  for (size_t i = 0; i < N; ++i) {
    std::cout << a[i] << (i + 1 < N ? ", " : "");
  }
  std::cout << "}}";
}
}  // namespace

int main() {
  std::cout << std::fixed << std::setprecision(6);
  try {
    franka::Robot robot(kRobotIp);
    franka::RobotState state = robot.readOnce();

    std::cout << "Robot mode: " << robotModeName(state.robot_mode) << "\n\n";

    std::cout << "q [rad] = ";
    printArray(state.q);
    std::cout << "\n\n";

    printTransform("O_T_EE (EE pose in base frame)", state.O_T_EE);
    std::cout << "O_T_EE (column-major) = ";
    printArray(state.O_T_EE);
    std::cout << "\n\n";

    // End-effector configuration as set in Desk.
    printTransform("F_T_EE (flange -> EE, from Desk)", state.F_T_EE);
    std::cout << "m_ee [kg] = " << state.m_ee << "\n";
    std::cout << "F_x_Cee [m] = ";
    printArray(state.F_x_Cee);
    std::cout << "\n\n";

    franka::Gripper gripper(kRobotIp);
    franka::GripperState gs = gripper.readOnce();
    std::cout << "Gripper: width=" << gs.width << " m, max_width=" << gs.max_width
              << " m, is_grasped=" << std::boolalpha << gs.is_grasped
              << ", temperature=" << gs.temperature << " C\n";
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
