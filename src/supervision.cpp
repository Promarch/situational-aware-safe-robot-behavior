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
    : model_(model), state_(state), params_(params), shared_(shared) {
  // F_T_tip = F_T_EE * translation(object_tip): the tip expressed along the EE's own axes.
  Eigen::Matrix4d F_T_tip = Eigen::Matrix4d::Map(state.F_T_EE.data());
  F_T_tip.block<3, 1>(0, 3) += F_T_tip.topLeftCorner<3, 3>() * params.object_tip;
  Eigen::Matrix4d::Map(F_T_tip_.data()) = F_T_tip;
}

Supervisor::Jacobian Supervisor::jacobian(const scurve::Vec7& q, bool at_object_tip) const {
  return Eigen::Map<const Jacobian>(model_
                                        .zeroJacobian(franka::Frame::kEndEffector, q,
                                                      flangeToReference(at_object_tip),
                                                      state_.EE_T_K)
                                        .data());
}

double Supervisor::effectiveMass(const scurve::Vec7& q, const Eigen::Vector3d& u,
                                 bool at_object_tip) const {
  const Jacobian J = jacobian(q, at_object_tip);
  const Matrix7 M = Eigen::Map<const Matrix7>(
      model_.mass(q, state_.I_total, state_.m_total, state_.F_x_Ctotal).data());
  const Eigen::Matrix<double, 6, 6> cartesian_mass_inverse = J * M.inverse() * J.transpose();
  return 1.0 / (u.transpose() * cartesian_mass_inverse.topLeftCorner<3, 3>() * u);
}

double Supervisor::tabulatedMass(const scurve::Vec7& q) const {
  const std::array<double, 16> T =
      model_.pose(franka::Frame::kEndEffector, q, state_.F_T_EE, state_.EE_T_K);
  return params_.constrained_mass->lookup(std::hypot(T[12], T[13]), T[14] - params_.table_z);
}

double Supervisor::limitMass(const scurve::Vec7& q, const Eigen::Vector3d& u,
                             bool at_object_tip, Contact contact) const {
  if (contact == Contact::kFree || params_.constrained_mass == nullptr) {
    return effectiveMass(q, u, at_object_tip);
  }
  if (contact == Contact::kConstrained) {
    return tabulatedMass(q);
  }
  return std::max(effectiveMass(q, u, at_object_tip), tabulatedMass(q));  // unknown
}

double Supervisor::safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u,
                             bool at_object_tip, Contact contact) const {
  return std::sqrt(2.0 * params_.s_lim / limitMass(q, u, at_object_tip, contact));
}

Eigen::Matrix<double, 7, 1> Supervisor::verticalJointRate(const scurve::Vec7& q,
                                                          double direction) const {
  const Jacobian J = jacobian(q, false);  // the EE is the frame the robot moves vertically
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  twist[2] = direction;
  return J.transpose() *
         (J * J.transpose() + kDamping * kDamping * Eigen::Matrix<double, 6, 6>::Identity())
             .ldlt()
             .solve(twist);
}

double Supervisor::evaluate(const Snapshot& s, const HandObservation& hands, bool* intrusion,
                            Contact* contact_out) const {
  // Joints at the predicted path parameter (sigma or vertical distance) over the horizon.
  // Joint motion: on the straight joint line. Vertical motion: follow the line from the
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

  // Reference point: the held object's far end, else the EE.
  const bool at_tip = shared_->holding_object.load();

  // Every hand against every predicted reference point, each with its own sphere radius. An
  // intruding hand near the table makes the contact constrained.
  bool intruding[kMaxHands] = {};
  for (const scurve::Vec7& qk : qs) {
    const std::array<double, 16> T = model_.pose(franka::Frame::kEndEffector, qk,
                                                 flangeToReference(at_tip), state_.EE_T_K);
    const Eigen::Vector3d p(T[12], T[13], T[14]);
    for (int h = 0; h < hands.n_hands; ++h) {
      const HandSphere& hand = hands.hands[h];
      intruding[h] |= (p - hand.center).norm() < hand.radius + params_.radius_margin;
    }
  }
  // Only a downward vertical motion can trap a hand against the table; every other motion
  // (joint motions in the air, moving up) is a free contact, also with the hand position unknown.
  Contact contact = Contact::kFree;
  if (s.vertical && s.direction < 0.0) {
    for (int h = 0; h < hands.n_hands; ++h) {
      if (intruding[h] && nearTable(hands.hands[h])) contact = Contact::kConstrained;
    }
    if (hands.unlocated) contact = Contact::kUnknown;
  }
  *contact_out = contact;
  *intrusion = hands.unlocated || std::any_of(intruding, intruding + hands.n_hands,
                                              [](bool b) { return b; });
  if (!*intrusion) {
    return s.nominal;
  }

  // Reference point velocity per unit of the motion's speed variable at each predicted pose:
  // J * delta for a joint motion, J * (joint rate for a unit vertical velocity) for a vertical
  // one. The cap is the highest speed at which every predicted point stays under its
  // energy-limited speed.
  double cap = s.nominal;
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta(s.delta.data());
  for (const scurve::Vec7& qk : qs) {
    const Eigen::Vector3d v_per_unit =
        jacobian(qk, at_tip).topRows<3>() *
        (s.vertical ? verticalJointRate(qk, s.direction) : delta);
    const double gain = v_per_unit.norm();
    if (gain > 1e-9) {
      cap = std::min(cap, safeSpeed(qk, v_per_unit / gain, at_tip, contact) / gain);
    }
  }
  return cap;
}

void Supervisor::send(const Snapshot& s, double cap, Contact contact) {
  Command c;
  c.motion_id = s.motion_id;
  c.seq = ++seq_;
  c.cap = cap;
  shared_->command.store(c);
  target_cap_ = cap;
  holding_ = cap < s.nominal * (1.0 - 1e-6);
  waiting_for_ = holding_ ? c.seq : 0;  // only a slowdown is waited for
  decisions_.push_back({s.motion_id, s.t, holding_, cap / s.nominal, contact});
}

void Supervisor::onFrame(const HandObservation& hands) {
  const Snapshot s = shared_->snapshot.load();
  if (s.motion_id == 0) {
    return;  // between motions
  }
  if (s.motion_id != motion_) {
    motion_ = s.motion_id;  // new motion starts at nominal speed
    waiting_for_ = 0;
    holding_ = false;
    target_cap_ = s.nominal;
  }
  if (waiting_for_ != 0) {
    // After a slowdown: computed once, look again only after it was applied, its change
    // finished and the reached speed was held for a while without looking. A resume is never
    // waited for, so a hand that intrudes while the robot speeds up again is still seen.
    if (s.applied_seq != waiting_for_ || s.changing) {
      return;
    }
    const double change_end = s.t_replan + scurve::velocityChangeEnd(s.profile);
    if (s.t < change_end + params_.hold_after_slowdown) {
      return;
    }
    waiting_for_ = 0;
  }

  bool intrusion = false;
  Contact contact = Contact::kFree;
  const bool any_hand = hands.n_hands > 0 || hands.unlocated;
  const double cap = any_hand ? evaluate(s, hands, &intrusion, &contact) : s.nominal;
  const double eps = 1e-6 * s.nominal;
  // Against the commanded cap, not s.cap: a resume not yet applied still shows the slow cap.
  if (intrusion && cap < target_cap_ - eps) {
    // Slow down. Also replaces a resume, whether still accelerating or not yet applied (the
    // control loop only sees the latest command).
    send(s, cap, contact);
  } else if (!intrusion && target_cap_ < s.nominal - eps) {
    send(s, s.nominal, Contact::kFree);  // resume
  }
}

}  // namespace supervision
