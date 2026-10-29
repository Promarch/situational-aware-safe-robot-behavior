#include "supervision.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace supervision {

namespace {
using Matrix7 = Eigen::Matrix<double, 7, 7>;
constexpr double kDamping = 1e-3;   // pseudo-inverse damping for the vertical prediction
constexpr double kLineStep = 2e-3;  // resolved-rate step along a vertical line [m]
}  // namespace

Supervisor::Supervisor(const franka::Model& model, const franka::RobotState& state,
                       const Params& params, Shared* shared)
    : model_(model), state_(state), params_(params), shared_(shared) {}

Supervisor::Jacobian Supervisor::jacobian(const scurve::Vec7& q) const {
  return Eigen::Map<const Jacobian>(
      model_.zeroJacobian(franka::Frame::kEndEffector, q, state_.F_T_EE, state_.EE_T_K).data());
}

double Supervisor::safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u) const {
  const Jacobian J = jacobian(q);
  const Matrix7 M = Eigen::Map<const Matrix7>(
      model_.mass(q, state_.I_total, state_.m_total, state_.F_x_Ctotal).data());
  const Eigen::Matrix<double, 6, 6> cartesian_mass_inverse = J * M.inverse() * J.transpose();
  const double effective_mass =
      1.0 / (u.transpose() * cartesian_mass_inverse.topLeftCorner<3, 3>() * u);
  return std::sqrt(2.0 * params_.s_lim / effective_mass);
}

Eigen::Matrix<double, 7, 1> Supervisor::verticalJointRate(const scurve::Vec7& q,
                                                          double direction) const {
  const Jacobian J = jacobian(q);
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  twist[2] = direction;
  return J.transpose() *
         (J * J.transpose() + kDamping * kDamping * Eigen::Matrix<double, 6, 6>::Identity())
             .ldlt()
             .solve(twist);
}

double Supervisor::evaluate(const Snapshot& s, const Eigen::Vector3d& hand,
                            bool* intrusion) const {
  // Joints at the predicted path parameter (sigma or vertical distance) over the horizon.
  // Joint segment: on the straight joint line. Vertical segment: follow the line from the
  // robot's current pose with the model, in small resolved-rate steps.
  std::vector<scurve::Vec7> qs;
  scurve::Vec7 q = s.q;
  double p_prev = s.profile.at(s.t - s.t_replan).p;
  for (double tau = 0.0; tau <= params_.horizon + 1e-9; tau += params_.sample_dt) {
    const double p = s.profile.at(s.t - s.t_replan + tau).p;
    if (s.vertical) {
      for (double remaining = p - p_prev; remaining > 1e-12; remaining -= kLineStep) {
        const double step = std::min(remaining, kLineStep);
        const Eigen::Matrix<double, 7, 1> dq = verticalJointRate(q, s.direction) * step;
        for (size_t i = 0; i < 7; ++i) q[i] += dq[i];
      }
      p_prev = p;
    } else {
      for (size_t i = 0; i < 7; ++i) q[i] = s.q_start[i] + s.delta[i] * p;
    }
    qs.push_back(q);
  }

  *intrusion = std::any_of(qs.begin(), qs.end(), [&](const scurve::Vec7& qk) {
    const std::array<double, 16> T =
        model_.pose(franka::Frame::kEndEffector, qk, state_.F_T_EE, state_.EE_T_K);
    return (Eigen::Vector3d(T[12], T[13], T[14]) - hand).norm() < params_.radius;
  });
  if (!*intrusion) {
    return s.nominal;
  }

  // EE velocity per unit of the segment's speed variable at each predicted pose: J * delta for a
  // joint segment, J * (joint rate for a unit vertical velocity) for a vertical one. The cap is
  // the highest speed at which every predicted point stays under its energy-limited speed.
  double cap = s.nominal;
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta(s.delta.data());
  for (const scurve::Vec7& qk : qs) {
    const Eigen::Vector3d v_per_unit =
        jacobian(qk).topRows<3>() * (s.vertical ? verticalJointRate(qk, s.direction) : delta);
    const double gain = v_per_unit.norm();
    if (gain > 1e-9) {
      cap = std::min(cap, safeSpeed(qk, v_per_unit / gain) / gain);
    }
  }
  return cap;
}

void Supervisor::send(const Snapshot& s, double cap) {
  Command c;
  c.segment_id = s.segment_id;
  c.seq = ++seq_;
  c.cap = cap;
  shared_->command.store(c);
  waiting_for_ = c.seq;
  holding_ = cap < s.nominal * (1.0 - 1e-6);
  decisions_.push_back({s.segment_id, s.t, holding_, cap / s.nominal});
}

void Supervisor::onFrame(const HandObservation& hand) {
  const Snapshot s = shared_->snapshot.load();
  if (s.segment_id == 0) {
    return;  // between segments
  }
  if (s.segment_id != segment_) {
    segment_ = s.segment_id;  // new segment starts at nominal speed
    waiting_for_ = 0;
  }
  if (waiting_for_ != 0) {
    // Computed once: look again only after our command was applied and its change finished,
    // and after a slowdown hold the reached speed for a while without looking.
    if (s.applied_seq != waiting_for_ || s.changing) {
      return;
    }
    const double change_end = s.t_replan + scurve::velocityChangeEnd(s.profile);
    if (holding_ && s.t < change_end + params_.hold_after_slowdown) {
      return;
    }
    waiting_for_ = 0;
  }

  bool intrusion = false;
  const double cap = hand.present ? evaluate(s, hand.position, &intrusion) : s.nominal;
  const double eps = 1e-6 * s.nominal;
  if (intrusion && cap < s.cap - eps) {
    send(s, cap);  // slow down
  } else if (!intrusion && s.cap < s.nominal - eps) {
    send(s, s.nominal);  // resume
  }
}

}  // namespace supervision
