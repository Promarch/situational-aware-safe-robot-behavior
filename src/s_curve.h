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
  size_t segmentCount() const { return count_; }
  const Segment& segment(size_t i) const { return segments_[i]; }

 private:
  std::array<Segment, kMaxSegments> segments_{};
  size_t count_ = 0;
  double duration_ = 0.0;
  State1D end_{};
};

// Rest-to-rest S-curve over distance L >= 0. Degenerates gracefully when v_max or a_max
// cannot be reached on a short distance. Always returns 7 segments (some may be zero length).
Profile1D planRestToRest(double L, double v_max, double a_max, double j_max);

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
  double duration() const { return sigma_.duration(); }
  const Profile1D& sigma() const { return sigma_; }

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
  Vec7 independent_durations_{};
  int limiting_v_ = -1;
  int limiting_a_ = -1;
  int limiting_j_ = -1;
};

}  // namespace scurve
