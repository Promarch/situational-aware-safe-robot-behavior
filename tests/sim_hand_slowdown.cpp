// Real-time simulation of the hand-aware slowdown with a virtual hand. The robot does NOT move:
// it is only read once for its model and load.
//
// Two threads, as on the robot:
//   control thread  - 1 kHz, runs the same JointSegment/VerticalSegment code as the robot
//                     callbacks, integrating the commanded velocities like the robot does
//   supervisor      - 30 Hz camera frames, Supervisor::onFrame() with the virtual hand
// Sequence: home -joint-> pre-pick -vertical-> grasp height (0.3 s pause) -vertical-> pre-pick
// -joint-> home.
//
// Usage: sim_hand_slowdown [SCHEDULE.csv | --random SEED] [S_lim in J, default 0.05]
//   writes sim_hand.csv (plot with scripts/plot_sim.py)
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "pick_sequence.h"
#include "poses.h"
#include "s_curve.h"
#include "segments.h"
#include "supervision.h"
#include "virtual_hand.h"

namespace {
using Clock = std::chrono::steady_clock;
using pick_sequence::eeOf;
using pick_sequence::kJointLimits;
using pick_sequence::kVerticalLimits;
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kDt = 1e-3;
constexpr double kFramePeriod = 1.0 / 30.0;
constexpr double kGripperPause = 0.3;  // stands in for the grasp [s]

enum SegmentIndex { kToPrePick, kDown, kUp, kToHome };

struct LogRow {
  double t;
  int segment;
  scurve::Vec7 q;
  Eigen::Vector3d ee;
  supervision::HandObservation hand;
};
}  // namespace

int main(int argc, char** argv) {
  supervision::Params params;
  // [SCHEDULE.csv | --random SEED] take one or two arguments; S_lim follows them.
  const int s_lim_arg = argc > 1 && std::string(argv[1]) == "--random" ? 3 : 2;
  if (argc > s_lim_arg) params.s_lim = std::atof(argv[s_lim_arg]);

  try {
    franka::Robot robot(kRobotIp);
    const franka::Model model = robot.loadModel();
    franka::RobotState state = robot.readOnce();  // only for F_T_EE, EE_T_K and load

    const scurve::JointSCurve to_pre(poses::kHome, poses::kPrePickJoints, kJointLimits);
    const double z_grasp = pick_sequence::kZGrasp;
    virtual_hand::Hand hand(
        pick_sequence::schedule(argc, argv, 1, pick_sequence::nominal(model, state)));
    std::cout << "S_lim " << params.s_lim << " J, radius " << params.radius * 100
              << " cm, horizon " << params.horizon << " s, hold after slowdown "
              << params.hold_after_slowdown << " s\n";

    // --- Threads.
    supervision::Shared shared;
    supervision::Supervisor supervisor(model, state, params, &shared);
    std::atomic<bool> done{false};
    const Clock::time_point t0 = Clock::now();
    auto now = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };

    std::thread camera([&] {
      Clock::time_point next = Clock::now();
      while (!done) {
        supervisor.onFrame(hand.observe(shared.snapshot.load(), now()), now());
        next += std::chrono::microseconds(static_cast<int>(kFramePeriod * 1e6));
        std::this_thread::sleep_until(next);
      }
    });

    std::vector<LogRow> log;
    log.reserve(200000);
    scurve::Vec7 q = poses::kHome;
    Clock::time_point tick = Clock::now();
    auto waitTick = [&] {
      tick += std::chrono::microseconds(1000);
      std::this_thread::sleep_until(tick);
    };
    auto runJoint = [&](int seg, const scurve::Vec7& goal) {
      motion::JointSegment segment(goal, kJointLimits, &shared);
      double t = 0.0;
      for (;; t += kDt) {
        const scurve::JointState ref = segment.step(t, q);
        log.push_back({now(), seg, q, eeOf(model, state, q), hand.at(now())});
        if (segment.finished(t)) break;
        for (size_t i = 0; i < 7; ++i) q[i] += ref.dq[i] * kDt;
        waitTick();
      }
      double err = 0.0;
      for (size_t i = 0; i < 7; ++i) err = std::max(err, std::abs(q[i] - goal[i]));
      std::cout << "[" << virtual_hand::kSegmentNames[seg] << "] ended after " << t
                << " s, |q - goal| = " << err * 1e3 << " mrad\n";
    };
    // Commanded EE pose, integrated from the vertical velocity commands as the robot does.
    std::array<double, 16> T{};
    auto runVertical = [&](int seg, double z_target) {
      motion::VerticalSegment segment(z_target, kVerticalLimits, &shared);
      for (double t = 0.0;; t += kDt) {
        const double vz = segment.step(t, T, q);
        log.push_back({now(), seg, q, eeOf(model, state, q), hand.at(now())});
        if (segment.finished(t)) break;
        T[14] += vz * kDt;
        // The robot solves IK for Cartesian velocity commands; follow the line with the model.
        const Eigen::Matrix<double, 7, 1> dq = supervisor.verticalJointRate(q, 1.0) * vz;
        for (size_t i = 0; i < 7; ++i) q[i] += dq[i] * kDt;
        waitTick();
      }
    };

    runJoint(kToPrePick, poses::kPrePickJoints);
    T = model.pose(franka::Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
    const double z_pre = T[14];
    runVertical(kDown, z_grasp);
    for (double t = 0.0; t < kGripperPause; t += kDt) waitTick();
    runVertical(kUp, z_pre);
    runJoint(kToHome, poses::kHome);
    done = true;
    camera.join();

    // --- Analysis.
    std::cout << "\nSupervisor decisions:\n";
    for (const std::string& e : supervisor.events()) std::cout << "  " << e << "\n";
    if (supervisor.events().empty()) std::cout << "  (none)\n";

    bool ok = true;
    auto check = [&](bool pass, const std::string& what) {
      std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] " << what << "\n";
      ok &= pass;
    };
    std::cout << "\nChecks:\n";

    // Limits as libfranka sees them: finite differences at 1 kHz within each segment.
    double joint_ratio[3] = {0, 0, 0}, vertical_ratio[3] = {0, 0, 0};
    for (size_t k = 3; k < log.size(); ++k) {
      if (log[k - 3].segment != log[k].segment) continue;
      if (log[k].segment == kDown || log[k].segment == kUp) {
        const auto z = [&](size_t m) { return log[m].ee.z(); };
        const double d[3] = {std::abs(z(k) - z(k - 1)) / kDt,
                             std::abs(z(k) - 2 * z(k - 1) + z(k - 2)) / (kDt * kDt),
                             std::abs(z(k) - 3 * z(k - 1) + 3 * z(k - 2) - z(k - 3)) /
                                 (kDt * kDt * kDt)};
        const double lim[3] = {kVerticalLimits.v, kVerticalLimits.a, kVerticalLimits.j};
        for (int n = 0; n < 3; ++n) vertical_ratio[n] = std::max(vertical_ratio[n], d[n] / lim[n]);
      } else {
        for (size_t i = 0; i < 7; ++i) {
          const auto qi = [&](size_t m) { return log[m].q[i]; };
          const double d[3] = {std::abs(qi(k) - qi(k - 1)) / kDt,
                               std::abs(qi(k) - 2 * qi(k - 1) + qi(k - 2)) / (kDt * kDt),
                               std::abs(qi(k) - 3 * qi(k - 1) + 3 * qi(k - 2) - qi(k - 3)) /
                                   (kDt * kDt * kDt)};
          const double lim[3] = {kJointLimits.v[i], kJointLimits.a[i], kJointLimits.j[i]};
          for (int n = 0; n < 3; ++n) joint_ratio[n] = std::max(joint_ratio[n], d[n] / lim[n]);
        }
      }
    }
    std::cout << "  joint segments: peak use of planned v/a/j limits " << joint_ratio[0] << " / "
              << joint_ratio[1] << " / " << joint_ratio[2] << "\n"
              << "  vertical segments: peak use of planned v/a/j limits " << vertical_ratio[0]
              << " / " << vertical_ratio[1] << " / " << vertical_ratio[2] << "\n";
    // Finite differences of a piecewise-cubic profile land slightly above the analytic peaks.
    check(joint_ratio[0] <= 1.001 && joint_ratio[1] <= 1.01 && joint_ratio[2] <= 1.05,
          "joint segments within limits");
    check(vertical_ratio[0] <= 1.001 && vertical_ratio[1] <= 1.01 && vertical_ratio[2] <= 1.05,
          "vertical segments within limits");

    double home_err = 0.0;
    for (size_t i = 0; i < 7; ++i) home_err = std::max(home_err, std::abs(q[i] - poses::kHome[i]));
    check(home_err < 1e-6, "back at home (" + std::to_string(home_err * 1e3) + " mrad)");

    // Wherever the EE actually came within the radius of the hand, compare its speed with the
    // energy-limited speed there. Right after a hand appears nothing can have slowed down yet:
    // allow one camera frame plus a full nominal-to-stop velocity change before judging. A hand
    // appearing closer than that is the "not enough distance" case we do not handle yet.
    auto stopTime = [](double v, double a, double j) {
      return scurve::planFromState({0.0, v, 0.0, 0.0}, 1e9, 0.0, a, j).duration();
    };
    const double react_vertical =
        kFramePeriod + stopTime(kVerticalLimits.v, kVerticalLimits.a, kVerticalLimits.j);
    const double react_joint =
        kFramePeriod + stopTime(to_pre.nominalSigmaVelocity(), to_pre.sigmaAccelerationLimit(),
                                to_pre.sigmaJerkLimit());
    std::cout << "  reaction time allowed after a hand appears: joint " << react_joint
              << " s, vertical " << react_vertical << " s\n";
    double worst_ratio = 0.0, worst_t = 0.0, worst_reacting = 0.0;
    int close_ticks = 0, reacting_ticks = 0;
    double appeared_at = -1.0;
    std::ofstream csv("sim_hand.csv");
    csv << std::setprecision(8)
        << "t,segment,x,y,z,speed,hand_present,hand_distance,v_safe\n";
    for (size_t k = 1; k < log.size(); ++k) {
      const LogRow& r = log[k];
      const Eigen::Vector3d v = (r.ee - log[k - 1].ee) / kDt;
      const double speed = log[k - 1].segment == r.segment ? v.norm() : 0.0;
      double dist = -1.0, v_safe = -1.0;
      if (!r.hand.present) {
        appeared_at = -1.0;
      } else {
        if (appeared_at < 0.0 || !log[k - 1].hand.present ||
            (r.hand.position - log[k - 1].hand.position).norm() > 1e-9) {
          appeared_at = r.t;
        }
        dist = (r.ee - r.hand.position).norm();
        if (speed > 1e-6) v_safe = supervisor.safeSpeed(r.q, v / speed);
        if (dist < params.radius && v_safe > 0.0) {
          const bool vertical = r.segment == kDown || r.segment == kUp;
          const bool reacting = r.t - appeared_at < (vertical ? react_vertical : react_joint);
          double& worst = reacting ? worst_reacting : worst_ratio;
          ++(reacting ? reacting_ticks : close_ticks);
          if (speed / v_safe > worst) {
            worst = speed / v_safe;
            if (!reacting) worst_t = r.t;
          }
        }
      }
      csv << r.t << "," << r.segment << "," << r.ee.x() << "," << r.ee.y() << "," << r.ee.z()
          << "," << speed << "," << r.hand.present << "," << dist << "," << v_safe << "\n";
    }
    std::cout << "  EE within " << params.radius * 100 << " cm of the hand:\n"
              << "    during the reaction time: " << reacting_ticks
              << " ms, worst speed / v_safe " << worst_reacting << " (not judged)\n"
              << "    after the reaction time:  " << close_ticks << " ms, worst speed / v_safe "
              << worst_ratio << (close_ticks ? " (t=" + std::to_string(worst_t) + " s)" : "")
              << "\n";
    check(worst_ratio <= 1.0 + 1e-3,
          "after reacting, EE never faster than v_safe while within the radius");
    std::cout << "\n" << (ok ? "SIMULATION PASSED" : "SIMULATION FAILED")
              << "  (plot: scripts/plot_sim.py sim_hand.csv)\n";
    return ok ? 0 : 1;
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << "\n";
    return -1;
  }
}
