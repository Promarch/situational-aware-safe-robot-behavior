// Offline test of the 7-axis S-curve. Does not connect to the robot.
//
// For each case it checks the profile analytically and as libfranka would see it at 1 kHz,
// prints a summary and writes s_curve_<case>.csv (t, sigma, q, dq, ddq, dddq).
//
// Usage: test_s_curve [fraction_v fraction_a fraction_j]
//   The optional fractions override the limits of the "custom" case (home -> pre-pick).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <franka/rate_limiting.h>

#include "poses.h"
#include "s_curve.h"

using scurve::JointLimits;
using scurve::JointSCurve;
using scurve::Vec7;

namespace {

constexpr double kDt = 1e-3;     // libfranka control period
constexpr double kTolRel = 1e-9;  // relative tolerance on analytic limits

// A real joint configuration recorded above the pick point (2026-10-26).
constexpr Vec7 kRecordedPrePick = {
    {-0.017395, 0.575934, -0.416741, -1.959414, 0.328241, 2.469573, 1.704578}};

struct Case {
  std::string name;
  Vec7 q_start;
  Vec7 q_goal;
  double fv, fa, fj;  // fractions of libfranka's hardware limits
  int expected_segments;  // 7: v and a reached, 6: only a, 5: only v, 4: neither, 0: no motion
};

// Number of non-degenerate phases in a rest-to-rest sigma profile.
int segmentPattern(const scurve::Profile1D& sigma) {
  if (sigma.segmentCount() == 0) return 0;
  const bool has_const_accel = sigma.segment(1).duration > 1e-12;
  const bool has_cruise = sigma.segment(3).duration > 1e-12;
  return 4 + (has_const_accel ? 2 : 0) + (has_cruise ? 1 : 0);
}

Vec7 offset(const Vec7& q, const Vec7& d) {
  Vec7 out;
  for (size_t i = 0; i < 7; ++i) out[i] = q[i] + d[i];
  return out;
}

bool check(bool ok, const char* what) {
  std::cout << "    [" << (ok ? "PASS" : "FAIL") << "] " << what << "\n";
  return ok;
}

bool runCase(const Case& c) {
  const JointLimits lim = scurve::frankaLimits(c.fv, c.fa, c.fj);
  const JointSCurve traj(c.q_start, c.q_goal, lim);
  const double T = traj.duration();

  std::cout << "\n=== " << c.name << "  (limits: v " << c.fv * 100 << "%, a " << c.fa * 100
            << "%, j " << c.fj * 100 << "%)\n";
  std::cout << "  synced duration: " << T << " s\n";
  std::cout << "  independent durations per joint [s]:";
  for (double d : traj.independentDurations()) std::cout << " " << d;
  std::cout << "\n  limiting joint (1-based) for v/a/j: " << traj.limitingJointV() + 1 << "/"
            << traj.limitingJointA() + 1 << "/" << traj.limitingJointJ() + 1 << "\n";
  std::cout << "  sigma phase durations [s]:";
  for (size_t i = 0; i < traj.sigma().segmentCount(); ++i) {
    std::cout << " " << traj.sigma().segment(i).duration;
  }
  const int pattern = segmentPattern(traj.sigma());
  std::cout << "\n  segment pattern: " << pattern << " (expected " << c.expected_segments
            << ")\n";

  // Sample at 1 kHz plus every segment boundary (where analytic extremes occur).
  std::vector<double> times;
  for (double t = 0.0; t < T; t += kDt) times.push_back(t);
  times.push_back(T);
  for (size_t i = 0; i < traj.sigma().segmentCount(); ++i) {
    times.push_back(traj.sigma().segment(i).t0);
  }

  Vec7 ratio_v{}, ratio_a{}, ratio_j{};
  for (double t : times) {
    const scurve::JointState s = traj.at(t);
    for (size_t i = 0; i < 7; ++i) {
      ratio_v[i] = std::max(ratio_v[i], std::abs(s.dq[i]) / lim.v[i]);
      ratio_a[i] = std::max(ratio_a[i], std::abs(s.ddq[i]) / lim.a[i]);
      ratio_j[i] = std::max(ratio_j[i], std::abs(s.dddq[i]) / lim.j[i]);
    }
  }

  // Continuity of sigma at segment boundaries. Sampling eps on each side, p/v/a legitimately
  // change by about j*2eps, so a real jump is anything clearly larger than that.
  constexpr double kEps = 1e-12;
  double max_abs_jerk = 0.0;
  for (size_t i = 0; i < traj.sigma().segmentCount(); ++i) {
    max_abs_jerk = std::max(max_abs_jerk, std::abs(traj.sigma().segment(i).j));
  }
  const double tol = 10.0 * max_abs_jerk * 2 * kEps + 1e-9;
  bool continuous = true;
  for (size_t i = 1; i < traj.sigma().segmentCount(); ++i) {
    const double tb = traj.sigma().segment(i).t0;
    const scurve::State1D before = traj.sigma().at(tb - kEps);
    const scurve::State1D after = traj.sigma().at(tb + kEps);
    continuous &= std::abs(after.p - before.p) < tol && std::abs(after.v - before.v) < tol &&
                  std::abs(after.a - before.a) < tol;
  }

  // What libfranka sees: finite differences of the commanded positions at 1 kHz.
  std::vector<Vec7> q_cmd;
  for (double t = 0.0; t <= T + 3 * kDt; t += kDt) q_cmd.push_back(traj.at(t).q);
  Vec7 fd_v{}, fd_a{}, fd_j{};
  for (size_t k = 3; k < q_cmd.size(); ++k) {
    for (size_t i = 0; i < 7; ++i) {
      const double v = (q_cmd[k][i] - q_cmd[k - 1][i]) / kDt;
      const double a = (q_cmd[k][i] - 2 * q_cmd[k - 1][i] + q_cmd[k - 2][i]) / (kDt * kDt);
      const double j = (q_cmd[k][i] - 3 * q_cmd[k - 1][i] + 3 * q_cmd[k - 2][i] - q_cmd[k - 3][i]) /
                       (kDt * kDt * kDt);
      fd_v[i] = std::max(fd_v[i], std::abs(v) / franka::kMaxJointVelocity[i]);
      fd_a[i] = std::max(fd_a[i], std::abs(a) / franka::kMaxJointAcceleration[i]);
      fd_j[i] = std::max(fd_j[i], std::abs(j) / franka::kMaxJointJerk[i]);
    }
  }

  const scurve::JointState end = traj.at(T);
  double end_err = 0.0, end_dq = 0.0, end_ddq = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    end_err = std::max(end_err, std::abs(end.q[i] - c.q_goal[i]));
    end_dq = std::max(end_dq, std::abs(end.dq[i]));
    end_ddq = std::max(end_ddq, std::abs(end.ddq[i]));
  }

  auto maxOf = [](const Vec7& r) { return *std::max_element(r.begin(), r.end()); };
  std::cout << "  peak usage of planned limits  v/a/j: " << maxOf(ratio_v) << " / "
            << maxOf(ratio_a) << " / " << maxOf(ratio_j) << "\n";
  std::cout << "  peak usage of HW limits @1kHz v/a/j: " << std::setprecision(9) << maxOf(fd_v)
            << " / " << maxOf(fd_a) << " / " << maxOf(fd_j) << std::setprecision(6) << "\n";

  bool ok = true;
  if (c.expected_segments >= 0) {
    ok &= check(pattern == c.expected_segments, "expected segment pattern");
  }
  ok &= check(end_err < 1e-9, "reaches goal (|q_end - q_goal| < 1e-9 rad)");
  ok &= check(end_dq < 1e-9 && end_ddq < 1e-9, "ends at rest (dq = ddq = 0)");
  ok &= check(maxOf(ratio_v) <= 1 + kTolRel && maxOf(ratio_a) <= 1 + kTolRel &&
                  maxOf(ratio_j) <= 1 + kTolRel,
              "all joints within planned v/a/j limits");
  ok &= check(continuous, "sigma continuous in p/v/a at segment boundaries");
  // Planning at 100% lands exactly on the limits; allow rounding noise only.
  ok &= check(maxOf(fd_v) <= 1 + 1e-6 && maxOf(fd_a) <= 1 + 1e-6 && maxOf(fd_j) <= 1 + 1e-6,
              "1 kHz command within libfranka hardware limits");

  std::ofstream csv("s_curve_" + c.name + ".csv");
  csv << std::setprecision(10) << "t,sigma";
  for (const char* p : {"q", "dq", "ddq", "dddq"}) {
    for (int i = 1; i <= 7; ++i) csv << "," << p << i;
  }
  csv << "\n";
  for (double t = 0.0; t <= T + kDt / 2; t += kDt) {
    const scurve::JointState s = traj.at(t);
    csv << t << "," << traj.sigma().at(t).p;
    for (const Vec7* v : {&s.q, &s.dq, &s.ddq, &s.dddq}) {
      for (double x : *v) csv << "," << x;
    }
    csv << "\n";
  }
  return ok;
}

}  // namespace

int main(int argc, char** argv) {
  double fv = 0.3, fa = 0.3, fj = 0.005;
  if (argc == 4) {
    fv = std::atof(argv[1]);
    fa = std::atof(argv[2]);
    fj = std::atof(argv[3]);
  } else if (argc != 1) {
    std::cerr << "usage: " << argv[0] << " [fraction_v fraction_a fraction_j]\n";
    return 2;
  }

  const Vec7& home = poses::kHome;
  // The custom case's pattern depends on the fractions given; 7 holds for the defaults.
  const int custom_pattern = argc == 4 ? -1 : 7;
  const std::vector<Case> cases = {
      {"custom", home, kRecordedPrePick, fv, fa, fj, custom_pattern},
      {"full_hw_limits", home, kRecordedPrePick, 1.0, 1.0, 1.0, 7},
      {"single_joint", home, offset(home, {{1.0, 0, 0, 0, 0, 0, 0}}), 0.3, 0.3, 0.005, 7},
      // v_max not reached, a_max reached: no cruise phase.
      {"seg6_no_vmax", home, offset(home, {{0.05, 0, 0, 0, 0, 0, 0}}), 0.3, 0.3, 0.1, 6},
      // v_max reached before a_max can be (v*j < a^2): no constant-acceleration phase.
      {"seg5_no_amax", home, offset(home, {{1.0, 0, 0, 0, 0, 0, 0}}), 0.3, 1.0, 0.001, 5},
      // Neither reached on a short move.
      {"seg4_short", home, offset(home, {{0.05, -0.03, 0, 0.04, 0, 0, 0}}), 0.3, 0.3, 0.005, 4},
      {"seg4_tiny", home, offset(home, {{0.002, 0, 0, -0.001, 0, 0, 0.003}}), 0.3, 0.3, 0.005, 4},
      // J1 is the slowest joint alone (velocity bound), but copying its phases would push J2
      // (a_max 7.5 vs 15) to 0.55 * 15 = 8.25 rad/s^2. Sync must stretch beyond J1's time.
      {"sync_other_axis_accel", home, offset(home, {{1.0, 0.55, 0, 0, 0, 0, 0}}), 1.0, 1.0, 1.0,
       7},
      {"no_motion", home, home, 0.3, 0.3, 0.005, 0},
  };

  bool all_ok = true;
  for (const Case& c : cases) all_ok &= runCase(c);
  std::cout << "\n" << (all_ok ? "ALL CASES PASSED" : "SOME CASES FAILED") << "\n";
  return all_ok ? 0 : 1;
}
