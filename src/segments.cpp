#include "segments.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
// Returns the pending command for this segment, if any newer than the last applied one.
bool pendingCap(supervision::Shared* shared, uint32_t id, uint32_t applied_seq,
                supervision::Command* out) {
  if (shared == nullptr) {
    return false;
  }
  *out = shared->command.load();
  return out->segment_id == id && out->seq > applied_seq;
}
}  // namespace

JointSegment::JointSegment(const scurve::Vec7& q_goal, const scurve::JointLimits& limits,
                           supervision::Shared* shared)
    : q_goal_(q_goal), limits_(limits), shared_(shared), traj_(q_goal, q_goal, limits) {
  if (shared_ != nullptr) {
    id_ = shared_->next_segment_id++;
  }
}

scurve::JointState JointSegment::step(double t, const scurve::Vec7& q_d) {
  if (!started_) {
    // Plan from the commanded position so the path starts exactly where the robot holds.
    traj_ = scurve::JointSCurve(q_d, q_goal_, limits_);
    cap_ = traj_.nominalSigmaVelocity();
    started_ = true;
  }
  supervision::Command c;
  if (pendingCap(shared_, id_, applied_seq_, &c)) {
    traj_.replan(t, c.cap);
    cap_ = std::min(c.cap, traj_.nominalSigmaVelocity());
    applied_seq_ = c.seq;
  }
  const scurve::JointState ref = traj_.at(t);

  if (shared_ != nullptr) {
    supervision::Snapshot s;
    s.segment_id = traj_.finished(t) ? 0 : id_;
    s.t = t;
    s.profile = traj_.sigma();
    s.t_replan = traj_.replanTime();
    s.changing = traj_.changingVelocity(t);
    s.applied_seq = applied_seq_;
    s.cap = cap_;
    s.nominal = traj_.nominalSigmaVelocity();
    s.q = ref.q;
    s.q_start = traj_.qStart();
    s.delta = traj_.delta();
    shared_->snapshot.store(s);
  }
  return ref;
}

VerticalSegment::VerticalSegment(double z_target, const CartesianLimits& limits,
                                 supervision::Shared* shared)
    : z_target_(z_target), limits_(limits), shared_(shared) {
  if (shared_ != nullptr) {
    id_ = shared_->next_segment_id++;
  }
}

bool VerticalSegment::finished(double t) const {
  return started_ && t - t_replan_ >= profile_.duration() &&
         profile_.end().p >= distance_ - 1e-9;
}

double VerticalSegment::step(double t, const std::array<double, 16>& O_T_EE_d,
                             const scurve::Vec7& q) {
  if (!started_) {
    // Plan from the commanded pose so the distance is exact.
    const double dz = z_target_ - O_T_EE_d[14];
    direction_ = dz >= 0.0 ? 1.0 : -1.0;
    distance_ = std::abs(dz);
    p_start_ = {O_T_EE_d[12], O_T_EE_d[13], O_T_EE_d[14]};
    profile_ = scurve::planRestToRest(distance_, limits_.v, limits_.a, limits_.j);
    cap_ = limits_.v;
    started_ = true;
  }
  supervision::Command c;
  if (pendingCap(shared_, id_, applied_seq_, &c)) {
    // The cap is the vertical speed itself: pull the target speed straight down.
    const scurve::State1D now = profile_.at(t - t_replan_);
    cap_ = std::min(c.cap, limits_.v);
    profile_ = scurve::planFromState(now, distance_, cap_, limits_.a, limits_.j);
    t_replan_ = t;
    replanned_ = true;
    applied_seq_ = c.seq;
  }
  const double v = profile_.at(t - t_replan_).v;

  if (shared_ != nullptr) {
    supervision::Snapshot s;
    s.segment_id = finished(t) ? 0 : id_;
    s.vertical = true;
    s.t = t;
    s.profile = profile_;
    s.t_replan = t_replan_;
    s.changing = replanned_ && t - t_replan_ < scurve::velocityChangeEnd(profile_);
    s.applied_seq = applied_seq_;
    s.cap = cap_;
    s.nominal = limits_.v;
    s.q = q;
    s.p_start = p_start_;
    s.direction = direction_;
    shared_->snapshot.store(s);
  }
  return direction_ * v;
}

}  // namespace motion
