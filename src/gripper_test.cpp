// Exercises the Franka Hand alone (the arm does not move) and prints the result of each
// command, to check how move() and grasp() behave with different parameters.
#include <iostream>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "confirm.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kSpeed = 0.05;  // [m/s]
constexpr double kForce = 15.0;  // [N]

void report(const char* what, bool ok, const franka::Gripper& gripper) {
  const franka::GripperState s = gripper.readOnce();
  std::cout << what << " -> returned " << std::boolalpha << ok << ", width " << s.width
            << " m, is_grasped " << s.is_grasped << "\n";
}
}  // namespace

int main() {
  try {
    franka::Gripper gripper(kRobotIp);
    const franka::GripperState s = gripper.readOnce();
    std::cout << "Start: width " << s.width << " m, max_width " << s.max_width << " m\n";

    if (!confirm("gripper only: homing, then move/grasp tests (keep the fingers free)")) {
      return 0;
    }
    report("homing", gripper.homing(), gripper);
    const double max_width = gripper.readOnce().max_width;

    report("move to 0.04 m", gripper.move(0.04, kSpeed), gripper);
    report("move to max_width", gripper.move(max_width, kSpeed), gripper);

    // Window (-0.005, 0.1) already contains the open width: does it still close?
    report("grasp(0, eps_in 0.005, eps_out 0.1)",
           gripper.grasp(0.0, kSpeed, kForce, 0.005, 0.1), gripper);
    report("move to max_width", gripper.move(max_width, kSpeed), gripper);

    // Window (-0.005, 0.005): only an empty, fully closed gripper succeeds.
    report("grasp(0, eps_in 0.005, eps_out 0.005)",
           gripper.grasp(0.0, kSpeed, kForce, 0.005, 0.005), gripper);
    report("move to max_width", gripper.move(max_width, kSpeed), gripper);
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
