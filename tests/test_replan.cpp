// Offline test of online S-curve re-planning (slow down / stop / resume). Does not connect to
// the robot.
//
// Each scenario runs home -> pre-pick at 1 kHz and calls JointSCurve::replan() at given times
// with given sigma-velocity caps (fractions of the nominal sigma velocity). It checks that the
// motion stays continuous, never reverses, keeps every joint within its limits, respects the
// cap after slowing down and still ends at rest on the goal. Writes replan_<scenario>.csv in
// the same format as test_s_curve, so plot_s_curve.py can plot it.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <franka/rate_limiting.h>

#include "poses.h"
#include "s_curve.h"

using scurve::JointSCurve;
using scurve::Vec7;

namespace {

constexpr double kDt = 1e-3;
constexpr double kTimeout = 30.0;   // [s]
constexpr double kSettle = 0.5;     // a velocity change with nominal limits is done by then [s]
constexpr double kTolRel = 1e-6;

struct Event {
  double t;    // [s]
  double cap;  // fraction of the nominal sigma velocity
};

struct Scenario {
  std::string name;
  std::vector<Event> events;
  bool ends_at_goal;
};

bool check(bool ok, const char* what) {
  std::cout << "    [" << (ok ? "PASS" : "FAIL") << "] " << what << "\n";
  return ok;
}

bool run(const Scenario& sc) {
  const scurve::JointLimits lim = scurve::frankaLimits(0.3, 0.3, 0.005);
  JointSCurve traj(poses::kHome, poses::kPrePickJoints, lim);
  const double v_nom = traj.nominalSigmaVelocity();
  std::cout << "\n=== " << sc.name << "  (nominal duration " << traj.duration() << " s)\n";

  struct Sample {
    double t;
    double sigma, sigma_dot;
    scurve::JointState js;
  };
  std::vector<Sample> samples;
  bool continuous = true, never_reverses = true, within_cap = true;
  Vec7 ratio_v{}, ratio_a{}, ratio_j{};
  size_t next_event = 0;
  double active_cap = 1.0, cap_since = 0.0;

  double t = 0.0;
  for (; t < kTimeout; t += kDt) {
    if (next_event < sc.events.size() && t >= sc.events[next_event].t) {
      const scurve::JointState before = traj.at(t);
      traj.replan(t, sc.events[next_event].cap * v_nom);
      const scurve::JointState after = traj.at(t);
      for (size_t i = 0; i < 7; ++i) {
        continuous &= std::abs(after.q[i] - before.q[i]) < 1e-9 &&
                      std::abs(after.dq[i] - before.dq[i]) < 1e-9 &&
                      std::abs(after.ddq[i] - before.ddq[i]) < 1e-9;
      }
      std::cout << "  t=" << t << " s: replan with cap " << sc.events[next_event].cap * 100
                << "% -> remaining " << traj.duration() - t << " s\n";
      active_cap = sc.events[next_event].cap;
      cap_since = t;
      ++next_event;
    }

    const scurve::State1D s = traj.sigma().at(t - traj.replanTime());
    const scurve::JointState js = traj.at(t);
    samples.push_back({t, s.p, s.v, js});
    never_reverses &= s.v >= -1e-12;
    if (t >= cap_since + kSettle) {
      within_cap &= s.v <= active_cap * v_nom * (1 + kTolRel) + 1e-12;
    }
    for (size_t i = 0; i < 7; ++i) {
      ratio_v[i] = std::max(ratio_v[i], std::abs(js.dq[i]) / lim.v[i]);
      ratio_a[i] = std::max(ratio_a[i], std::abs(js.ddq[i]) / lim.a[i]);
      ratio_j[i] = std::max(ratio_j[i], std::abs(js.dddq[i]) / lim.j[i]);
    }
    if (traj.finished(t) || (next_event == sc.events.size() && t >= traj.duration())) {
      break;
    }
  }

  // What libfranka sees: finite differences of the 1 kHz positions.
  Vec7 fd_v{}, fd_a{}, fd_j{};
  for (size_t k = 3; k < samples.size(); ++k) {
    for (size_t i = 0; i < 7; ++i) {
      const auto q = [&](size_t m) { return samples[m].js.q[i]; };
      fd_v[i] = std::max(fd_v[i], std::abs(q(k) - q(k - 1)) / kDt / franka::kMaxJointVelocity[i]);
      fd_a[i] = std::max(fd_a[i], std::abs(q(k) - 2 * q(k - 1) + q(k - 2)) / (kDt * kDt) /
                                      franka::kMaxJointAcceleration[i]);
      fd_j[i] = std::max(fd_j[i],
                         std::abs(q(k) - 3 * q(k - 1) + 3 * q(k - 2) - q(k - 3)) /
                             (kDt * kDt * kDt) / franka::kMaxJointJerk[i]);
    }
  }

  const scurve::JointState end = traj.at(t);
  double goal_err = 0.0, end_speed = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    goal_err = std::max(goal_err, std::abs(end.q[i] - poses::kPrePickJoints[i]));
    end_speed = std::max({end_speed, std::abs(end.dq[i]), std::abs(end.ddq[i])});
  }
  auto maxOf = [](const Vec7& r) { return *std::max_element(r.begin(), r.end()); };
  double peak_after_last = 0.0;
  for (const Sample& s : samples) {
    if (!sc.events.empty() && s.t >= sc.events.back().t) {
      peak_after_last = std::max(peak_after_last, s.sigma_dot);
    }
  }
  std::cout << "  peak speed after last replan: " << peak_after_last / v_nom * 100
            << "% of nominal\n";
  std::cout << "  total " << t << " s, final sigma " << samples.back().sigma
            << "\n  peak usage of planned limits  v/a/j: " << maxOf(ratio_v) << " / "
            << maxOf(ratio_a) << " / " << maxOf(ratio_j)
            << "\n  peak usage of HW limits @1kHz v/a/j: " << maxOf(fd_v) << " / " << maxOf(fd_a)
            << " / " << maxOf(fd_j) << "\n";

  bool ok = true;
  ok &= check(continuous, "q/dq/ddq continuous at every replan");
  ok &= check(never_reverses, "never moves backwards along the path");
  ok &= check(within_cap, "speed stays under the active cap once the change has settled");
  ok &= check(maxOf(ratio_v) <= 1 + kTolRel && maxOf(ratio_a) <= 1 + kTolRel &&
                  maxOf(ratio_j) <= 1 + kTolRel,
              "all joints within planned v/a/j limits");
  ok &= check(maxOf(fd_v) <= 1 && maxOf(fd_a) <= 1 && maxOf(fd_j) <= 1,
              "1 kHz command within libfranka hardware limits");
  ok &= check(end_speed < 1e-9, "ends at rest");
  if (sc.ends_at_goal) {
    ok &= check(goal_err < 1e-9, "ends on the goal (|q - q_goal| < 1e-9 rad)");
  } else {
    ok &= check(samples.back().sigma < 1.0 - 1e-6, "stopped on the path before the goal");
  }

  std::ofstream csv("replan_" + sc.name + ".csv");
  csv << std::setprecision(10) << "t,sigma";
  for (const char* p : {"q", "dq", "ddq", "dddq"}) {
    for (int i = 1; i <= 7; ++i) csv << "," << p << i;
  }
  csv << "\n";
  for (const Sample& s : samples) {
    csv << s.t << "," << s.sigma;
    for (const Vec7* v : {&s.js.q, &s.js.dq, &s.js.ddq, &s.js.dddq}) {
      for (double x : *v) csv << "," << x;
    }
    csv << "\n";
  }
  return ok;
}

}  // namespace

int main() {
  // Nominal home -> pre-pick: ~2.2 s, accelerating until ~0.4 s, cruising until ~1.8 s.
  const std::vector<Scenario> scenarios = {
      {"slow_in_cruise", {{1.0, 0.3}}, true},
      {"slow_while_accelerating", {{0.15, 0.3}}, true},
      {"slow_then_resume", {{0.6, 0.3}, {1.6, 1.0}}, true},
      {"stop_then_resume", {{0.8, 0.0}, {2.0, 1.0}}, true},
      {"stop_on_path", {{0.8, 0.0}}, false},
      // Crawls at 10% almost to the end, then resumes: too close for full speed, so the
      // cruise speed must be lowered to still stop exactly on the goal.
      {"resume_near_end", {{0.5, 0.1}, {12.5, 1.0}}, true},
      // Resume while already in the final deceleration: there is no room to hold the current
      // speed, so the profile must keep decelerating and still stop exactly on the goal.
      {"resume_in_final_decel", {{1.0, 0.6}, {2.45, 1.0}}, true},
      {"resume_in_final_decel_late", {{1.0, 0.6}, {2.65, 1.0}}, true},
  };
  bool all_ok = true;
  for (const Scenario& sc : scenarios) all_ok &= run(sc);
  std::cout << "\n" << (all_ok ? "ALL SCENARIOS PASSED" : "SOME SCENARIOS FAILED") << "\n";
  return all_ok ? 0 : 1;
}
