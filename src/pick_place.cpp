// Pick-and-place task. Step 1: move to home with libfranka's example MotionGenerator.
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "poses.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kSpeedFactor = 0.2;
}

int main() {
  try {
    franka::Robot robot(kRobotIp);
    setDefaultBehavior(robot);

    std::cout << "WARNING: robot will move to home. Keep the user stop at hand.\n"
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    MotionGenerator to_home(kSpeedFactor, poses::kHome);
    robot.control(to_home);
    std::cout << "Reached home." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
