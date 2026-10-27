// Fixed poses used by the pick-and-place task.
#pragma once

#include <array>
#include <cmath>

namespace poses {

using JointPose = std::array<double, 7>;
using CartesianPose = std::array<double, 16>;  // column-major 4x4, same layout as O_T_EE

// Franka standard "ready" pose.
constexpr JointPose kHome = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

// Joint configuration recorded with read_state above the pick point on 2026-10-26
// (EE at x=0.560 y=-0.255 z=0.097, tool axis ~1 deg from vertical). Used as a reset pose
// until IK for kPrePick is available.
constexpr JointPose kPrePickJoints = {
    {-0.017395, 0.575934, -0.416741, -1.959414, 0.328241, 2.469573, 1.704578}};

// Height of the table surface in the base frame [m].
constexpr double kTableZ = 0.0;

// Above the pick point. x/y taken from the pose recorded with read_state on 2026-10-26
// (Franka Hand configured in Desk: F_T_EE z=0.1034, m_ee=0.73 kg).
// Orientation snapped to the base axes: EE z points straight down at the table,
// EE y (finger motion direction) along base -x, EE x along base -y.
constexpr double kPickX = 0.560397;
constexpr double kPickY = -0.254549;
constexpr double kPrePickHeight = 0.20;  // above the table

constexpr CartesianPose kPrePick = {{0.0, -1.0, 0.0, 0.0,    // EE x axis
                                     -1.0, 0.0, 0.0, 0.0,    // EE y axis
                                     0.0, 0.0, -1.0, 0.0,    // EE z axis
                                     kPickX, kPickY, kTableZ + kPrePickHeight, 1.0}};

}  // namespace poses
