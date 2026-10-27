#include "motion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Geometry>

namespace motion {

JointMotionResult moveJointSCurve(franka::Robot& robot, const scurve::Vec7& q_goal,
                                  const scurve::JointLimits& limits) {
  // The robot integrates our dq into q_d. Plan from q_d at the first tick so the path starts
  // exactly where the robot is holding.
  std::optional<scurve::JointSCurve> traj;
  JointMotionResult result;
  franka::RobotState last_state;
  robot.control([&](const franka::RobotState& state,
                    franka::Duration period) -> franka::JointVelocities {
    result.duration += period.toSec();
    if (!traj) {
      traj.emplace(state.q_d, q_goal, limits);
    }
    const scurve::JointState ref = traj->at(result.duration);
    for (size_t i = 0; i < 7; ++i) {
      result.max_lag = std::max(result.max_lag, std::abs(ref.q[i] - state.q_d[i]));
    }
    last_state = state;
    franka::JointVelocities cmd(ref.dq);
    if (result.duration >= traj->duration()) {
      return franka::MotionFinished(cmd);
    }
    return cmd;
  });

  for (size_t i = 0; i < 7; ++i) {
    result.end_error = std::max(result.end_error, std::abs(last_state.q_d[i] - q_goal[i]));
  }
  return result;
}

VerticalMotionResult moveVerticalTo(franka::Robot& robot, double z_target,
                                    const CartesianLimits& limits) {
  std::optional<scurve::Profile1D> profile;
  double direction = 1.0;
  double x0 = 0.0, y0 = 0.0;
  VerticalMotionResult result;
  franka::RobotState last_state;
  robot.control([&](const franka::RobotState& state,
                    franka::Duration period) -> franka::CartesianVelocities {
    result.duration += period.toSec();
    if (!profile) {
      // Plan from the commanded pose so the distance is exact.
      const double dz = z_target - state.O_T_EE_d[14];
      direction = dz >= 0.0 ? 1.0 : -1.0;
      profile = scurve::planRestToRest(std::abs(dz), limits.v, limits.a, limits.j);
      x0 = state.O_T_EE[12];
      y0 = state.O_T_EE[13];
    }
    result.max_xy_deviation =
        std::max(result.max_xy_deviation,
                 std::hypot(state.O_T_EE[12] - x0, state.O_T_EE[13] - y0));
    last_state = state;
    const double vz = direction * profile->at(result.duration).v;
    franka::CartesianVelocities cmd(std::array<double, 6>{{0.0, 0.0, vz, 0.0, 0.0, 0.0}});
    if (result.duration >= profile->duration()) {
      return franka::MotionFinished(cmd);
    }
    return cmd;
  });

  result.z_d_end = last_state.O_T_EE_d[14];
  result.z_end = last_state.O_T_EE[14];
  return result;
}

double moveCartesianPoseTo(franka::Robot& robot, const std::array<double, 16>& O_T_target,
                           const CartesianLimits& translation, const CartesianLimits& rotation) {
  const Eigen::Affine3d T1(Eigen::Matrix4d::Map(O_T_target.data()));
  const Eigen::Vector3d p1 = T1.translation();
  const Eigen::Quaterniond q1(T1.linear());

  std::optional<scurve::Profile1D> sigma;
  Eigen::Vector3d p0;
  Eigen::Quaterniond q0;
  double time = 0.0;
  robot.control([&](const franka::RobotState& state,
                    franka::Duration period) -> franka::CartesianPose {
    time += period.toSec();
    if (!sigma) {
      // Plan from the commanded pose; sigma's limits are the tightest of translation and
      // rotation, each divided by the distance it has to cover.
      const Eigen::Affine3d T0(Eigen::Matrix4d::Map(state.O_T_EE_d.data()));
      p0 = T0.translation();
      q0 = Eigen::Quaterniond(T0.linear());
      if (q0.dot(q1) < 0.0) {
        q0.coeffs() *= -1.0;  // take the short way around
      }
      const double dist = (p1 - p0).norm();
      const double angle = q0.angularDistance(q1);
      constexpr double kInf = std::numeric_limits<double>::infinity();
      auto bound = [&](double lin, double rot) {
        return std::min(dist > 1e-9 ? lin / dist : kInf, angle > 1e-9 ? rot / angle : kInf);
      };
      const double v = bound(translation.v, rotation.v);
      sigma = std::isinf(v) ? scurve::planRestToRest(0.0, 1.0, 1.0, 1.0)
                            : scurve::planRestToRest(1.0, v, bound(translation.a, rotation.a),
                                                     bound(translation.j, rotation.j));
    }
    const double s = sigma->segmentCount() > 0 ? sigma->at(time).p : 1.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = p0 + s * (p1 - p0);
    T.linear() = q0.slerp(s, q1).normalized().toRotationMatrix();

    std::array<double, 16> pose;
    Eigen::Matrix4d::Map(pose.data()) = T.matrix();
    franka::CartesianPose cmd(pose);
    if (time >= sigma->duration()) {
      return franka::MotionFinished(cmd);
    }
    return cmd;
  });
  return time;
}

}  // namespace motion
