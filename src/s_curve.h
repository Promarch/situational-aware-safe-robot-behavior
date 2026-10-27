// Jerk-limited (7-segment) S-curve profiles.
//
// A 1-D profile is a sequence of constant-jerk segments. The 7-axis profile drives all joints
// with one normalized profile sigma(t) in [0, 1]:
//   q_i(t) = q_start_i + delta_i * sigma(t)
// so every joint shares the same phase durations and the joint-space path is a straight line.
#pragma once

#include <array>
#include <cstddef>

namespace scurve {

using Vec7 = std::array<double, 7>;

struct State1D {
  double p = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
};

class Profile1D {
 public:
  static constexpr size_t kMaxSegments = 7;

  struct Segment {
    double t0;        // start time
    double duration;
    double p0, v0, a0;  // state at segment start
    double j;         // constant jerk inside the segment
  };

  Profile1D() = default;

  // Builds a profile from an initial state and a list of (duration, jerk) pairs.
  Profile1D(const State1D& start, const std::array<double, kMaxSegments>& durations,
            const std::array<double, kMaxSegments>& jerks, size_t count);

  // State at time t; t is clamped to [0, duration()].
  State1D at(double t) const;

  double duration() const { return duration_; }
  State1D end() const { return end_; }
  size_t segmentCount() const { return count_; }
  const Segment& segment(size_t i) const { return segments_[i]; }

 private:
  std::array<Segment, kMaxSegments> segments_{};
  size_t count_ = 0;
  double duration_ = 0.0;
  State1D end_{};
};

// End time of the leading velocity change of a profile built by planFromState (its first three
// segments), relative to the profile's start.
double velocityChangeEnd(const Profile1D& profile);

// Rest-to-rest S-curve over distance L >= 0. Degenerates gracefully when v_max or a_max
// cannot be reached on a short distance. Always returns 7 segments (some may be zero length).
Profile1D planRestToRest(double L, double v_max, double a_max, double j_max);

// From an arbitrary moving state to rest at p_end: change velocity to a cruise speed of at
// most v_cap, cruise, then stop exactly at p_end. Every acceleration change uses a_max/j_max,
// i.e. the same shape as a rest-to-rest profile's acceleration phases. If p_end is too close to
// reach v_cap, the cruise speed is lowered so the stop still lands on p_end. v_cap <= 0 plans
// only the stop, which then ends before p_end. Assumes start.v >= 0 and enough distance to stop.
Profile1D planFromState(const State1D& start, double p_end, double v_cap, double a_max,
                        double j_max);

struct JointLimits {
  Vec7 v;  // [rad/s]
  Vec7 a;  // [rad/s^2]
  Vec7 j;  // [rad/s^3]
};

// libfranka's hardware limits (franka/rate_limiting.h) scaled by the given fractions.
JointLimits frankaLimits(double fraction_v, double fraction_a, double fraction_j);

struct JointState {
  Vec7 q{}, dq{}, ddq{}, dddq{};
};

class JointSCurve {
 public:
  JointSCurve(const Vec7& q_start, const Vec7& q_goal, const JointLimits& limits);

  JointState at(double t) const;
  double duration() const { return t_replan_ + sigma_.duration(); }
  // True once the motion has come to rest at q_goal (not merely stopped on the path).
  bool finished(double t) const;

  // Re-plans sigma from its state at time t with the same synchronized limits, so every joint
  // stays within its limits and the path is unchanged. The new cruise speed is at most
  // sigma_dot_cap (nominalSigmaVelocity() to resume, a lower value to slow down, 0 to stop on
  // the path). The motion still ends at rest at q_goal unless sigma_dot_cap is 0.
  void replan(double t, double sigma_dot_cap);
  double nominalSigmaVelocity() const { return v_sigma_; }
  double sigmaAccelerationLimit() const { return a_sigma_; }
  double sigmaJerkLimit() const { return j_sigma_; }

  // True while the velocity change started by the last replan() is still running.
  bool changingVelocity(double t) const;

  // Current sigma profile; its time 0 is the last replan time.
  const Profile1D& sigma() const { return sigma_; }
  double replanTime() const { return t_replan_; }

  // Joint path: q = qStart() + delta() * sigma.
  const Vec7& qStart() const { return q_start_; }
  const Vec7& delta() const { return delta_; }

  // Duration each joint would need if it were planned alone with its own limits.
  const Vec7& independentDurations() const { return independent_durations_; }

  // Joints that bound sigma's velocity, acceleration and jerk limits (-1 if no motion).
  int limitingJointV() const { return limiting_v_; }
  int limitingJointA() const { return limiting_a_; }
  int limitingJointJ() const { return limiting_j_; }

 private:
  Vec7 q_start_;
  Vec7 delta_;
  Profile1D sigma_;
  double t_replan_ = 0.0;
  bool replanned_ = false;
  double v_sigma_ = 0.0, a_sigma_ = 0.0, j_sigma_ = 0.0;  // synchronized sigma limits
  Vec7 independent_durations_{};
  int limiting_v_ = -1;
  int limiting_a_ = -1;
  int limiting_j_ = -1;
};

}  // namespace scurve
