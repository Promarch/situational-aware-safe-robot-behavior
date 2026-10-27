#include "supervision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <Eigen/Dense>

namespace supervision {

namespace {
using Matrix7 = Eigen::Matrix<double, 7, 7>;
constexpr double kDamping = 1e-3;   // pseudo-inverse damping for the vertical prediction
constexpr double kLineStep = 2e-3;  // resolved-rate step along a vertical line [m]

scurve::Vec7 pathPoint(const Snapshot& s, double sigma) {
  scurve::Vec7 q;
  for (size_t i = 0; i < 7; ++i) q[i] = s.q_start[i] + s.delta[i] * sigma;
  return q;
}
}  // namespace

Supervisor::Supervisor(const franka::Model& model, const franka::RobotState& state,
                       const Params& params, Shared* shared)
    : model_(model), state_(state), params_(params), shared_(shared) {}

double Supervisor::safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u) const {
  const Jacobian J = jacobian(q);
  const Matrix7 M = Eigen::Map<const Matrix7>(
      model_.mass(q, state_.I_total, state_.m_total, state_.F_x_Ctotal).data());
  const Eigen::Matrix<double, 6, 6> cartesian_mass_inverse = J * M.inverse() * J.transpose();
  const double effective_mass =
      1.0 / (u.transpose() * cartesian_mass_inverse.topLeftCorner<3, 3>() * u);
  return std::sqrt(2.0 * params_.s_lim / effective_mass);
}

Supervisor::Jacobian Supervisor::jacobian(const scurve::Vec7& q) const {
  return Eigen::Map<const Jacobian>(
      model_.zeroJacobian(franka::Frame::kEndEffector, q, state_.F_T_EE, state_.EE_T_K).data());
}

Eigen::Matrix<double, 7, 1> Supervisor::verticalJointRate(const scurve::Vec7& q,
                                                          double direction) const {
  // Joint velocity producing a unit vertical EE velocity with no rotation (damped pseudo-inverse).
  const Jacobian J = jacobian(q);
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  twist[2] = direction;
  return J.transpose() *
         (J * J.transpose() + kDamping * kDamping * Eigen::Matrix<double, 6, 6>::Identity())
             .ldlt()
             .solve(twist);
}

std::vector<scurve::Vec7> Supervisor::predictJoints(const Snapshot& s,
                                                    const std::vector<double>& path) const {
  std::vector<scurve::Vec7> qs;
  if (!s.vertical) {
    for (double p : path) qs.push_back(pathPoint(s, p));
    return qs;
  }
  // Vertical segment: follow the line from the robot's current pose with the model, in small
  // resolved-rate steps (path[0] is the current position on the line).
  scurve::Vec7 q = s.q;
  double p_prev = path.front();
  for (double p : path) {
    for (double remaining = p - p_prev; remaining > 1e-12;) {
      const double step = std::min(remaining, kLineStep);
      const Eigen::Matrix<double, 7, 1> dq = verticalJointRate(q, s.direction) * step;
      for (size_t i = 0; i < 7; ++i) q[i] += dq[i];
      remaining -= step;
    }
    p_prev = p;
    qs.push_back(q);
  }
  return qs;
}

double Supervisor::evaluate(const Snapshot& s, const Eigen::Vector3d& hand,
                            bool* intrusion) const {
  // Predicted path parameter (sigma or vertical distance) over the horizon, and the joints there.
  std::vector<double> path;
  for (double tau = 0.0; tau <= params_.horizon + 1e-9; tau += params_.sample_dt) {
    path.push_back(s.profile.at(s.t - s.t_replan + tau).p);
  }
  const std::vector<scurve::Vec7> qs = predictJoints(s, path);

  *intrusion = false;
  for (const scurve::Vec7& q : qs) {
    const std::array<double, 16> T =
        model_.pose(franka::Frame::kEndEffector, q, state_.F_T_EE, state_.EE_T_K);
    if ((Eigen::Vector3d(T[12], T[13], T[14]) - hand).norm() < params_.radius) {
      *intrusion = true;
      break;
    }
  }
  if (!*intrusion) {
    return s.nominal;
  }

  // EE velocity per unit of the segment's speed variable, from the model at each predicted pose:
  // J * delta for a joint segment (per unit sigma-velocity), J * (joint rate for a unit vertical
  // velocity) for a vertical one. The cap is the highest speed at which every predicted point
  // stays under its energy-limited speed.
  double cap = s.nominal;
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta(s.delta.data());
  for (const scurve::Vec7& q : qs) {
    const Jacobian J = jacobian(q);
    const Eigen::Vector3d v_per_unit =
        J.topRows<3>() * (s.vertical ? verticalJointRate(q, s.direction) : delta);
    const double gain = v_per_unit.norm();
    if (gain > 1e-9) {
      cap = std::min(cap, safeSpeed(q, v_per_unit / gain) / gain);
    }
  }
  return cap;
}

void Supervisor::send(const Snapshot& s, double cap, double now, const std::string& why) {
  Command c;
  c.segment_id = s.segment_id;
  c.seq = ++seq_;
  c.cap = cap;
  shared_->command.store(c);
  waiting_for_ = c.seq;

  const scurve::State1D now_state = s.profile.at(s.t - s.t_replan);
  const char* phase = now_state.a > 1e-9 ? "accelerating" : now_state.a < -1e-9 ? "decelerating"
                                                                                  : "cruising";
  std::ostringstream line;
  line.precision(4);
  line << "t=" << now << " s  segment " << s.segment_id << (s.vertical ? " (vertical)" : " (joint)")
       << ", " << phase << " at " << now_state.v / s.nominal * 100 << "% -> " << why << ", cap "
       << cap / s.nominal * 100 << "%";
  std::lock_guard<std::mutex> lock(log_mutex_);
  log_.push_back(line.str());
}

void Supervisor::onFrame(const HandObservation& hand, double now) {
  const Snapshot s = shared_->snapshot.load();
  if (s.segment_id == 0) {
    return;  // between segments
  }
  if (s.segment_id != segment_) {
    segment_ = s.segment_id;  // new segment starts at nominal speed
    waiting_for_ = 0;
  }
  if (waiting_for_ != 0) {
    // Computed once: look again only after our command was applied and its change finished.
    if (s.applied_seq != waiting_for_ || s.changing) {
      return;
    }
    waiting_for_ = 0;
  }

  bool intrusion = false;
  const double cap = hand.present ? evaluate(s, hand.position, &intrusion) : s.nominal;
  const double kEps = 1e-6 * s.nominal;
  if (intrusion && cap < s.cap - kEps) {
    send(s, cap, now, "hand intrudes, slow down");
  } else if (!intrusion && s.cap < s.nominal - kEps) {
    send(s, s.nominal, now, hand.present ? "hand clear of the path, resume" : "hand gone, resume");
  }
}

std::vector<std::string> Supervisor::events() const {
  std::lock_guard<std::mutex> lock(log_mutex_);
  return log_;
}

}  // namespace supervision
