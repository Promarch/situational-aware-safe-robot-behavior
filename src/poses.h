// Fixed poses used by the pick-and-place task.
#pragma once

#include <array>
#include <cmath>

namespace poses {

using JointPose = std::array<double, 7>;

// Franka standard "ready" pose.
constexpr JointPose kHome = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

// Start and end of the slowdown test: home with joint 1 turned +90 deg (EE on the robot's left),
// so the joint segments to and from the pick point (on the right) are long.
constexpr JointPose kStart = {{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

// 20 cm above the pick point (x=0.560 y=-0.255), tool axis straight down, fingers aligned with
// the base axes. Recorded on 2026-10-27 after reaching that pose through the Cartesian pose
// interface (Franka Hand in Desk: F_T_EE z=0.1034, m_ee=0.73 kg).
constexpr JointPose kPrePickJoints = {
    {-0.035886, 0.377146, -0.416845, -1.943431, 0.197729, 2.280601, 1.799564}};

// Above the place point (the pen holder), the gripper turned 90 deg so the screwdriver (held
// along the EE x axis) hangs straight down: EE x axis straight down, tool z axis horizontal, EE
// kPrePlaceHeight above the table. Recorded on 2026-11-02 with
//   record_pose kPrePlaceJoints --x-down --z 0.30
constexpr JointPose kPrePlaceJoints = {
    {2.323794, 0.779593, -0.398577, -1.461532, -1.080884, 1.352486, -0.167051}};

// Height of the table surface in the base frame [m].
constexpr double kTableZ = 0.0;

// EE (fingertip center) above the table when grasping [m].
constexpr double kGraspHeight = 0.02;

// EE (fingertip center) above the table at the pre-place pose [m]; the screwdriver's tip hangs
// 0.15 m lower, clear of the holder's rim.
constexpr double kPrePlaceHeight = 0.30;

// Straight descent from the pre-place pose into the holder before releasing [m].
constexpr double kPlaceDescent = 0.10;

}  // namespace poses
