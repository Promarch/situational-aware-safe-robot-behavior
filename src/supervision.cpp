#include "supervision.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace supervision {

namespace {
constexpr double kDamping = 1e-3;   // pseudo-inverse damping for the vertical prediction
constexpr double kLineStep = 2e-3;  // resolved-rate step along a vertical line [m]
constexpr double kMinGain = 1e-9;   // contact points slower than this per unit speed are skipped

robot_body::BodyConfig bodyConfig(const Params& p) {
  robot_body::BodyConfig config;
  config.object_tip = p.object_tip;
  config.object_radius = p.object_radius;
  config.check_links = p.check_links;
  config.franka_hand = p.franka_hand;
  config.safety_distance = p.link_safety_distance;
  return config;
}
}  // namespace

Supervisor::Supervisor(const franka::Model& model, const franka::RobotState& state,
                       const Params& params, Shared* shared)
    : model_(model),
      state_(state),
      params_(params),
      shared_(shared),
      body_(model, state, bodyConfig(params)) {
  // F_T_tip = F_T_EE * translation(object_tip): the tip expressed along the EE's own axes.
  Eigen::Matrix4d F_T_tip = Eigen::Matrix4d::Map(state.F_T_EE.data());
  F_T_tip.block<3, 1>(0, 3) += F_T_tip.topLeftCorner<3, 3>() * params.object_tip;
  Eigen::Matrix4d::Map(F_T_tip_.data()) = F_T_tip;
  target_.cap = 0.0;
}

Supervisor::Jacobian Supervisor::jacobian(const scurve::Vec7& q, bool at_object_tip) const {
  return Eigen::Map<const Jacobian>(model_
                                        .zeroJacobian(franka::Frame::kEndEffector, q,
                                                      flangeToReference(at_object_tip),
                                                      state_.EE_T_K)
                                        .data());
}

Supervisor::Matrix7 Supervisor::massInverse(const scurve::Vec7& q) const {
  return Eigen::Map<const Matrix7>(
             model_.mass(q, state_.I_total, state_.m_total, state_.F_x_Ctotal).data())
      .inverse();
}

double Supervisor::effectiveMass(const scurve::Vec7& q, const Eigen::Vector3d& u,
                                 bool at_object_tip) const {
  const Eigen::Matrix<double, 3, 7> Jv = jacobian(q, at_object_tip).topRows<3>();
  return 1.0 / (u.transpose() * Jv * massInverse(q) * Jv.transpose() * u);
}

double Supervisor::tabulatedMass(const scurve::Vec7& q) const {
  const std::array<double, 16> T =
      model_.pose(franka::Frame::kEndEffector, q, state_.F_T_EE, state_.EE_T_K);
  return params_.constrained_mass->lookup(std::hypot(T[12], T[13]), T[14] - params_.table_z);
}

double Supervisor::limitMass(const scurve::Vec7& q, const Eigen::Vector3d& u,
                             bool at_object_tip, Contact contact) const {
  // The reference point is on the tool.
  return limitMassAt(q, effectiveMass(q, u, at_object_tip), robot_body::kHand, contact);
}

double Supervisor::limitMassAt(const scurve::Vec7& q, double m_eff, uint8_t capsule,
                               Contact contact) const {
  if (contact == Contact::kFree || params_.constrained_mass == nullptr) {
    return m_eff;
  }
  const double constrained = robot_body::isTool(capsule)
                                 ? tabulatedMass(q)
                                 : std::max(m_eff, params_.constrained_mass->maxMass());
  return contact == Contact::kConstrained ? constrained : std::max(m_eff, constrained);
}

double Supervisor::safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u,
                             bool at_object_tip, Contact contact) const {
  return std::sqrt(2.0 * params_.s_lim / limitMass(q, u, at_object_tip, contact));
}

robot_body::Nearest Supervisor::clearance(const std::vector<robot_body::Capsule>& capsules,
                                          const HandSphere& hand) const {
  robot_body::Nearest n = robot_body::nearestCapsule(capsules, hand.center);
  n.distance -= hand.radius + params_.radius_margin;
  return n;
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

std::vector<scurve::Vec7> Supervisor::predict(const Snapshot& s) const {
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
  return qs;
}

Eigen::Vector3d Supervisor::referencePoint(const scurve::Vec7& q, bool at_object_tip) const {
  const std::array<double, 16> T = model_.pose(franka::Frame::kEndEffector, q,
                                               flangeToReference(at_object_tip), state_.EE_T_K);
  return Eigen::Vector3d(T[12], T[13], T[14]);
}

Supervisor::View Supervisor::view(const Snapshot& s, bool holding,
                                  const HandObservation& hands) const {
  // Every hand against every capsule at every predicted sample, each hand with its own sphere
  // radius. Only a downward vertical motion can trap a hand against the table; every other motion
  // (joint motions in the air, moving up) is a free contact.
  View v;
  for (const scurve::Vec7& q : predict(s)) {
    Sample sample{q, body_.frames(q), {}};
    sample.capsules = body_.capsules(sample.frames, holding);
    v.samples.push_back(std::move(sample));
  }
  const bool moving_down = s.vertical && s.direction < 0.0;
  for (int h = 0; h < hands.n_hands; ++h) {
    const HandSphere& hand = hands.hands[h];
    HandAssessment& a = v.hands[h];
    a.clearance = std::numeric_limits<double>::infinity();
    for (const Sample& sample : v.samples) {
      for (const robot_body::Capsule& c : sample.capsules) {
        const double t = robot_body::closestOnSegment(c.a, c.b, hand.center);
        const double clear = (hand.center - robot_body::pointOn(c, t)).norm() - c.radius -
                             hand.radius - params_.radius_margin;
        if (clear < 0.0) a.exposed |= static_cast<uint16_t>(1u << c.id);
        if (clear < a.clearance) {
          a.clearance = clear;
          a.capsule = c.id;
          a.closest = robot_body::surfacePoint(c, t, hand.center);
        }
      }
    }
    if (a.exposed != 0) {
      a.status = moving_down && nearTable(hand) ? HandStatus::kConstrained : HandStatus::kIntruding;
    }
  }
  return v;
}

Supervisor::Evaluation Supervisor::evaluate(const Snapshot& s, const View& v,
                                            const HandObservation& hands) const {
  Evaluation e;
  e.cap = s.nominal;
  bool any_exposed = false;
  for (int h = 0; h < hands.n_hands; ++h) {
    any_exposed |= v.hands[h].exposed != 0;
    if (v.hands[h].status == HandStatus::kConstrained) e.contact = Contact::kConstrained;
  }
  const bool moving_down = s.vertical && s.direction < 0.0;
  // An unknown hand position may be anywhere: every capsule is exposed.
  const Contact unlocated_contact = moving_down ? Contact::kUnknown : Contact::kFree;
  if (hands.unlocated) e.contact = std::max(e.contact, unlocated_contact);
  e.intrusion = hands.unlocated || any_exposed;
  if (!e.intrusion) {
    return e;
  }

  // At every predicted sample, the possible contact points on every exposed capsule's axis: for a
  // located hand the axis point closest to it and both ends of the stretch of the axis inside its
  // intrusion sphere; for an unlocated hand both ends of every capsule. Axis points, not surface
  // points: towards the hand (along the surface normal) the facing surface point moves exactly as
  // fast and has exactly the same effective mass; the surface only adds sliding (omega x r n),
  // which cannot press into a hand. Each point's velocity per unit of the motion's speed variable
  // is J_p * delta for a joint motion, J_p * (joint rate for a unit vertical velocity) for a
  // vertical one. The cap is the highest speed at which every point stays under its
  // energy-limited speed.
  struct Candidate {
    Eigen::Vector3d p;
    Contact contact;
  };
  std::vector<Candidate> candidates;
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta(s.delta.data());
  for (const Sample& sample : v.samples) {
    const Eigen::Matrix<double, 7, 1> rate =
        s.vertical ? verticalJointRate(sample.q, s.direction) : Eigen::Matrix<double, 7, 1>(delta);
    bool have_m_inverse = false;
    Matrix7 m_inverse;
    // libfranka's Jacobian of each frame that carries a candidate, once per sample.
    std::array<robot_body::Jacobian, robot_body::kNumFrames> frame_jacobian;
    std::array<bool, robot_body::kNumFrames> have_frame_jacobian{};
    for (const robot_body::Capsule& c : sample.capsules) {
      candidates.clear();
      if (hands.unlocated) {
        candidates.push_back({c.a, unlocated_contact});
        candidates.push_back({c.b, unlocated_contact});
      }
      for (int h = 0; h < hands.n_hands; ++h) {
        if ((v.hands[h].exposed & (1u << c.id)) == 0) continue;
        const HandSphere& hand = hands.hands[h];
        const Contact contact =
            moving_down && nearTable(hand) ? Contact::kConstrained : Contact::kFree;
        candidates.push_back(
            {robot_body::pointOn(c, robot_body::closestOnSegment(c.a, c.b, hand.center)), contact});
        double t0, t1;
        if (robot_body::segmentChord(c.a, c.b, hand.center,
                                     hand.radius + c.radius + params_.radius_margin, &t0, &t1)) {
          candidates.push_back({robot_body::pointOn(c, t0), contact});
          candidates.push_back({robot_body::pointOn(c, t1), contact});
        }
      }
      if (!candidates.empty() && !have_frame_jacobian[c.frame]) {
        frame_jacobian[c.frame] = body_.frameJacobian(sample.q, c.frame);
        have_frame_jacobian[c.frame] = true;
      }
      for (const Candidate& k : candidates) {
        const robot_body::LinearJacobian Jv = robot_body::shiftToPoint(
            frame_jacobian[c.frame], k.p - sample.frames[c.frame].translation());
        const Eigen::Vector3d v_per_unit = Jv * rate;
        const double gain = v_per_unit.norm();
        if (gain < kMinGain) continue;
        if (!have_m_inverse) {
          m_inverse = massInverse(sample.q);
          have_m_inverse = true;
        }
        const Eigen::Vector3d u = v_per_unit / gain;
        const double m_eff = 1.0 / (u.transpose() * Jv * m_inverse * Jv.transpose() * u);
        const double m = limitMassAt(sample.q, m_eff, c.id, k.contact);
        const double cap = std::sqrt(2.0 * params_.s_lim / m) / gain;
        if (cap < e.cap) {
          e.cap = cap;
          e.contact = k.contact;
          e.capsule = c.id;
          e.point = k.p;
        }
      }
    }
  }
  return e;
}

void Supervisor::send(const Snapshot& s, const Evaluation& e) {
  Command c;
  c.motion_id = s.motion_id;
  c.seq = ++seq_;
  c.cap = e.cap;
  shared_->command.store(c);
  target_ = e;
  holding_ = e.cap < s.nominal * (1.0 - 1e-6);
  waiting_for_ = holding_ ? c.seq : 0;  // only a slowdown is waited for
  decisions_.push_back({s.motion_id, s.t, holding_, e.cap / s.nominal,
                        holding_ ? e.contact : Contact::kFree,
                        holding_ ? e.capsule : robot_body::kNoCapsule});
}

void Supervisor::onFrame(const HandObservation& hands) {
  const auto t0 = std::chrono::steady_clock::now();
  const Snapshot s = shared_->snapshot.load();
  const bool holding = shared_->holding_object.load();
  const View v = s.motion_id != 0 ? view(s, holding, hands) : View{};
  decide(s, v, hands);
  if (overlay_sink_) overlay_sink_(overlay(s, holding, v, hands));
  max_frame_time_ = std::max(
      max_frame_time_,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}

Overlay Supervisor::overlay(const Snapshot& s, bool holding, const View& v,
                            const HandObservation& hands) const {
  Overlay o;
  // Before the first motion the snapshot is still the default one.
  o.valid = std::any_of(s.q.begin(), s.q.end(), [](double x) { return x != 0.0; });
  if (!o.valid) return o;
  o.holding_object = holding;
  o.reference = referencePoint(s.q, holding);
  o.capsules = body_.capsules(s.q, holding);
  o.hands = hands;
  o.moving = s.motion_id != 0;
  if (!o.moving) {
    // At rest nothing can be hit: every hand is clear, with its clearance to the robot as it is.
    for (int h = 0; h < hands.n_hands; ++h) {
      const robot_body::Nearest n = clearance(o.capsules, hands.hands[h]);
      if (n.index < 0) continue;
      o.assessment[h].capsule = o.capsules[n.index].id;
      o.assessment[h].clearance = n.distance;
      o.assessment[h].closest = n.point;
    }
    return o;
  }
  const Eigen::Vector3d tip = holding ? params_.object_tip : Eigen::Vector3d::Zero();
  for (const Sample& sample : v.samples) {
    o.predicted.push_back(sample.frames[robot_body::kEndEffector] * tip);
  }
  o.assessment = v.hands;
  if (s.motion_id == motion_) {
    o.slowed = holding_;
    o.contact = target_.contact;
    o.cap_fraction = target_.cap / s.nominal;
    if (holding_) {
      o.limiting_capsule = target_.capsule;
      o.limiting_point = target_.point;
    }
  }
  return o;
}

void Supervisor::decide(const Snapshot& s, const View& v, const HandObservation& hands) {
  if (s.motion_id == 0) {
    return;  // between motions
  }
  if (s.motion_id != motion_) {
    motion_ = s.motion_id;  // new motion starts at nominal speed
    waiting_for_ = 0;
    holding_ = false;
    target_ = Evaluation{};
    target_.cap = s.nominal;
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

  const bool any_hand = hands.n_hands > 0 || hands.unlocated;
  Evaluation e;
  e.cap = s.nominal;
  if (any_hand) e = evaluate(s, v, hands);
  const double eps = 1e-6 * s.nominal;
  // Against the commanded cap, not s.cap: a resume not yet applied still shows the slow cap.
  if (e.intrusion && e.cap < target_.cap - eps) {
    // Slow down. Also replaces a resume, whether still accelerating or not yet applied (the
    // control loop only sees the latest command).
    send(s, e);
  } else if (!e.intrusion && target_.cap < s.nominal - eps) {
    Evaluation resume;
    resume.cap = s.nominal;
    send(s, resume);
  }
}

}  // namespace supervision
