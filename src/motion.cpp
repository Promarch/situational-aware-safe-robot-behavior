#include "motion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Geometry>

namespace motion {

namespace {
// The supervisor's latest command for segment `id`, if newer than `applied_seq`. A single read
// attempt: never waits in the control loop.
bool pendingCap(const supervision::Shared* shared, uint32_t id, uint32_t applied_seq,
                supervision::Command* c) {
  return shared != nullptr && shared->command.tryLoad(c) && c->segment_id == id &&
         c->seq > applied_seq;
}
}  // namespace

double moveJointSCurve(franka::Robot& robot, const scurve::Vec7& q_goal,
                       const scurve::JointLimits& limits, supervision::Shared* shared,
                       TickLog* log) {
  const uint32_t id = shared != nullptr ? shared->next_segment_id++ : 0;
  std::optional<scurve::JointSCurve> traj;
  uint32_t applied_seq = 0;
  double cap = 0.0;
  double t = 0.0;
  franka::RobotState last;
  robot.control([&](const franka::RobotState& state,
                    franka::Duration period) -> franka::JointVelocities {
    t += period.toSec();
    if (!traj) {
      // The robot integrates our dq into q_d: start the path exactly where it is holding.
      traj.emplace(state.q_d, q_goal, limits);
      cap = traj->nominalSigmaVelocity();
    }
    supervision::Command c;
    if (pendingCap(shared, id, applied_seq, &c)) {
      traj->replan(t, c.cap);
      cap = std::min(c.cap, traj->nominalSigmaVelocity());
      applied_seq = c.seq;
    }
    const scurve::JointState ref = traj->at(t);
    const bool finished = traj->finished(t);

    if (shared != nullptr) {
      supervision::Snapshot s;
      s.segment_id = finished ? 0 : id;
      s.t = t;
      s.profile = traj->sigma();
      s.t_replan = traj->replanTime();
      s.changing = traj->changingVelocity(t);
      s.applied_seq = applied_seq;
      s.cap = cap;
      s.nominal = traj->nominalSigmaVelocity();
      s.q = ref.q;
      s.q_start = traj->qStart();
      s.delta = traj->delta();
      shared->snapshot.store(s);
    }
    if (log != nullptr) {
      log->push({log->now(), static_cast<int>(id) - 1, t, state.q, state.dq, state.dq_d,
                 state.tau_ext_hat_filtered, shared != nullptr && shared->holding_object.load()});
    }
    last = state;
    franka::JointVelocities cmd(ref.dq);
    return finished ? franka::MotionFinished(cmd) : cmd;
  });

  double end_error = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    end_error = std::max(end_error, std::abs(last.q_d[i] - q_goal[i]));
  }
  return end_error;
}

double moveVerticalTo(franka::Robot& robot, double z_target, const CartesianLimits& limits,
                      supervision::Shared* shared, TickLog* log) {
  const uint32_t id = shared != nullptr ? shared->next_segment_id++ : 0;
  bool started = false;
  double distance = 0.0, direction = 1.0;
  scurve::Profile1D profile;  // travelled distance; its time 0 is t_replan
  double t_replan = 0.0;
  bool replanned = false;
  uint32_t applied_seq = 0;
  double cap = limits.v;
  double t = 0.0;
  franka::RobotState last;
  robot.control([&](const franka::RobotState& state,
                    franka::Duration period) -> franka::CartesianVelocities {
    t += period.toSec();
    if (!started) {
      // Plan from the commanded pose so the distance is exact.
      const double dz = z_target - state.O_T_EE_d[14];
      direction = dz >= 0.0 ? 1.0 : -1.0;
      distance = std::abs(dz);
      profile = scurve::planRestToRest(distance, limits.v, limits.a, limits.j);
      started = true;
    }
    supervision::Command c;
    if (pendingCap(shared, id, applied_seq, &c)) {
      // The cap is the vertical speed itself: pull the target speed straight down.
      cap = std::min(c.cap, limits.v);
      profile = scurve::planFromState(profile.at(t - t_replan), distance, cap, limits.a, limits.j);
      t_replan = t;
      replanned = true;
      applied_seq = c.seq;
    }
    const double vz = direction * profile.at(t - t_replan).v;
    const bool finished =
        t - t_replan >= profile.duration() && profile.end().p >= distance - 1e-9;

    if (shared != nullptr) {
      supervision::Snapshot s;
      s.segment_id = finished ? 0 : id;
      s.vertical = true;
      s.t = t;
      s.profile = profile;
      s.t_replan = t_replan;
      s.changing = replanned && t - t_replan < scurve::velocityChangeEnd(profile);
      s.applied_seq = applied_seq;
      s.cap = cap;
      s.nominal = limits.v;
      s.q = state.q;
      s.direction = direction;
      shared->snapshot.store(s);
    }
    if (log != nullptr) {
      log->push({log->now(), static_cast<int>(id) - 1, t, state.q, state.dq, state.dq_d,
                 state.tau_ext_hat_filtered, shared != nullptr && shared->holding_object.load()});
    }
    last = state;
    franka::CartesianVelocities cmd(std::array<double, 6>{{0.0, 0.0, vz, 0.0, 0.0, 0.0}});
    return finished ? franka::MotionFinished(cmd) : cmd;
  });
  return last.O_T_EE[14];
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
