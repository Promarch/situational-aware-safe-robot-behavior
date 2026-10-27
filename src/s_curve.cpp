#include "s_curve.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <franka/rate_limiting.h>

namespace scurve {

namespace {
constexpr double kMinDelta = 1e-9;  // joint displacements below this count as no motion

using Durations = std::array<double, Profile1D::kMaxSegments>;

// Appends up to 3 segments taking (v0, a0) to (v1, 0): jerk towards a peak acceleration
// (at most a_max), hold it, jerk back to zero.
void appendVelocityChange(double v0, double a0, double v1, double a_max, double j_max,
                          Durations* durations, Durations* jerks, size_t* n) {
  // Velocity reached by just bringing a0 back to zero decides the direction.
  const double v_ramp = v0 + a0 * std::abs(a0) / (2.0 * j_max);
  const double d = v1 >= v_ramp ? 1.0 : -1.0;
  const double A = d * a0;          // initial acceleration in the direction of change
  const double dv = d * (v1 - v0);  // velocity change in that direction

  // Peak P: the change covers (2P^2 - A^2) / (2j) plus P * T_hold.
  double P = a_max;
  double T_hold = (dv - (2.0 * a_max * a_max - A * A) / (2.0 * j_max)) / a_max;
  if (T_hold < 0.0) {
    T_hold = 0.0;
    P = std::sqrt(std::max(0.0, (2.0 * j_max * dv + A * A) / 2.0));
  }
  const double phases[3] = {(P - A) / j_max, T_hold, P / j_max};
  const double phase_jerks[3] = {d * j_max, 0.0, -d * j_max};
  for (int i = 0; i < 3; ++i) {
    (*durations)[*n] = std::max(0.0, phases[i]);
    (*jerks)[*n] = phase_jerks[i];
    ++*n;
  }
}

// Velocity change to vp, cruise for T_cruise, then stop.
Profile1D changeCruiseStop(const State1D& start, double vp, double T_cruise, double a_max,
                           double j_max) {
  Durations durations{}, jerks{};
  size_t n = 0;
  appendVelocityChange(start.v, start.a, vp, a_max, j_max, &durations, &jerks, &n);
  durations[n] = T_cruise;
  jerks[n] = 0.0;
  ++n;
  appendVelocityChange(vp, 0.0, 0.0, a_max, j_max, &durations, &jerks, &n);
  return Profile1D(start, durations, jerks, n);
}
}  // namespace

Profile1D::Profile1D(const State1D& start, const std::array<double, kMaxSegments>& durations,
                     const std::array<double, kMaxSegments>& jerks, size_t count)
    : count_(count) {
  double t = 0.0, p = start.p, v = start.v, a = start.a;
  for (size_t i = 0; i < count_; ++i) {
    const double T = std::max(0.0, durations[i]);
    const double j = jerks[i];
    segments_[i] = {t, T, p, v, a, j};
    p += v * T + a * T * T / 2.0 + j * T * T * T / 6.0;
    v += a * T + j * T * T / 2.0;
    a += j * T;
    t += T;
  }
  duration_ = t;
  end_ = {p, v, a, 0.0};
}

State1D Profile1D::at(double t) const {
  if (count_ == 0) {
    return end_;
  }
  if (t >= duration_) {
    return end_;
  }
  t = std::max(t, 0.0);
  size_t i = 0;
  while (i + 1 < count_ && t >= segments_[i + 1].t0) {
    ++i;
  }
  const Segment& s = segments_[i];
  const double tau = t - s.t0;
  State1D out;
  out.p = s.p0 + s.v0 * tau + s.a0 * tau * tau / 2.0 + s.j * tau * tau * tau / 6.0;
  out.v = s.v0 + s.a0 * tau + s.j * tau * tau / 2.0;
  out.a = s.a0 + s.j * tau;
  out.j = s.j;
  return out;
}

Profile1D planRestToRest(double L, double v_max, double a_max, double j_max) {
  if (L <= 0.0) {
    return Profile1D(State1D{}, {}, {}, 0);
  }

  // Durations of the jerk phase and constant-acceleration phase to reach peak velocity vp.
  auto accelPhases = [&](double vp, double* Tj, double* Tca) {
    if (vp * j_max >= a_max * a_max) {
      *Tj = a_max / j_max;
      *Tca = vp / a_max - *Tj;
    } else {
      *Tj = std::sqrt(vp / j_max);  // a_max not reached
      *Tca = 0.0;
    }
  };

  double vp = v_max;
  double Tj, Tca;
  accelPhases(vp, &Tj, &Tca);
  // Accelerating to vp and back to rest covers vp * (2*Tj + Tca).
  double Tv = (L - vp * (2.0 * Tj + Tca)) / vp;

  if (Tv < 0.0) {
    // v_max not reached: solve for the peak velocity that covers exactly L.
    Tv = 0.0;
    vp = (-a_max * a_max / j_max +
          std::sqrt(std::pow(a_max, 4) / (j_max * j_max) + 4.0 * a_max * L)) /
         2.0;
    if (vp * j_max < a_max * a_max) {
      vp = std::cbrt(L * L * j_max / 4.0);  // a_max not reached either
    }
    accelPhases(vp, &Tj, &Tca);
  }

  const std::array<double, 7> durations{{Tj, Tca, Tj, Tv, Tj, Tca, Tj}};
  const std::array<double, 7> jerks{{j_max, 0.0, -j_max, 0.0, -j_max, 0.0, j_max}};
  return Profile1D(State1D{}, durations, jerks, 7);
}

double velocityChangeEnd(const Profile1D& profile) {
  constexpr size_t kChangeSegments = 3;
  return profile.segmentCount() > kChangeSegments ? profile.segment(kChangeSegments).t0
                                                  : profile.duration();
}

Profile1D planFromState(const State1D& start, double p_end, double v_cap, double a_max,
                        double j_max) {
  if (v_cap <= 0.0) {
    Durations durations{}, jerks{};
    size_t n = 0;
    appendVelocityChange(start.v, start.a, 0.0, a_max, j_max, &durations, &jerks, &n);
    return Profile1D(start, durations, jerks, n);
  }

  const double distance = p_end - start.p;
  auto covered = [&](double vp) {
    return changeCruiseStop(start, vp, 0.0, a_max, j_max).end().p - start.p;
  };

  double vp = v_cap;
  if (covered(vp) > distance) {
    // Not enough room for v_cap: bisect the highest cruise speed that still stops in time.
    // The lower bound is the current speed, which needs no extra room to reach.
    double lo = std::min(start.v, v_cap), hi = v_cap;
    for (int i = 0; i < 60; ++i) {
      const double mid = 0.5 * (lo + hi);
      (covered(mid) > distance ? hi : lo) = mid;
    }
    vp = lo;
  }
  const double T_cruise = vp > 0.0 ? std::max(0.0, (distance - covered(vp)) / vp) : 0.0;
  return changeCruiseStop(start, vp, T_cruise, a_max, j_max);
}

JointLimits frankaLimits(double fraction_v, double fraction_a, double fraction_j) {
  JointLimits l;
  for (size_t i = 0; i < 7; ++i) {
    l.v[i] = franka::kMaxJointVelocity[i] * fraction_v;
    l.a[i] = franka::kMaxJointAcceleration[i] * fraction_a;
    l.j[i] = franka::kMaxJointJerk[i] * fraction_j;
  }
  return l;
}

JointSCurve::JointSCurve(const Vec7& q_start, const Vec7& q_goal, const JointLimits& limits)
    : q_start_(q_start) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  double v_sigma = kInf, a_sigma = kInf, j_sigma = kInf;

  for (int i = 0; i < 7; ++i) {
    delta_[i] = q_goal[i] - q_start[i];
    const double d = std::abs(delta_[i]);
    if (d < kMinDelta) {
      continue;
    }
    independent_durations_[i] = planRestToRest(d, limits.v[i], limits.a[i], limits.j[i]).duration();

    // Joint i moves delta_i * sigma, so sigma's limits are the joint limits divided by |delta_i|.
    // Taking the minimum per derivative keeps every joint within all three of its limits.
    if (limits.v[i] / d < v_sigma) {
      v_sigma = limits.v[i] / d;
      limiting_v_ = i;
    }
    if (limits.a[i] / d < a_sigma) {
      a_sigma = limits.a[i] / d;
      limiting_a_ = i;
    }
    if (limits.j[i] / d < j_sigma) {
      j_sigma = limits.j[i] / d;
      limiting_j_ = i;
    }
  }

  if (limiting_v_ >= 0) {
    v_sigma_ = v_sigma;
    a_sigma_ = a_sigma;
    j_sigma_ = j_sigma;
    sigma_ = planRestToRest(1.0, v_sigma_, a_sigma_, j_sigma_);
  }
}

bool JointSCurve::finished(double t) const {
  return t >= duration() && (sigma_.segmentCount() == 0 || sigma_.end().p >= 1.0 - 1e-9);
}

void JointSCurve::replan(double t, double sigma_dot_cap) {
  if (sigma_.segmentCount() == 0) {
    return;  // no motion
  }
  const State1D now = sigma_.at(t - t_replan_);
  sigma_ = planFromState(now, 1.0, std::min(sigma_dot_cap, v_sigma_), a_sigma_, j_sigma_);
  t_replan_ = t;
  replanned_ = true;
}

bool JointSCurve::changingVelocity(double t) const {
  return replanned_ && t - t_replan_ < velocityChangeEnd(sigma_);
}

JointState JointSCurve::at(double t) const {
  const State1D s = sigma_.at(t - t_replan_);
  // With no motion sigma_ is empty; keep the joints at the start.
  const double p = sigma_.segmentCount() > 0 ? s.p : 0.0;
  JointState out;
  for (size_t i = 0; i < 7; ++i) {
    out.q[i] = q_start_[i] + delta_[i] * p;
    out.dq[i] = delta_[i] * s.v;
    out.ddq[i] = delta_[i] * s.a;
    out.dddq[i] = delta_[i] * s.j;
  }
  return out;
}

}  // namespace scurve
