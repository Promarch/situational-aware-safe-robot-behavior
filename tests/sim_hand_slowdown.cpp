// Real-time simulation of the hand-aware slowdown with a simulated hand. The robot does NOT move:
// it is only read once for its model and load.
//
// Two threads, as on the robot:
//   control thread  - 1 kHz, runs the same JointSegment/VerticalSegment code as the robot
//                     callbacks, integrating the commanded velocities like the robot does
//   supervisor      - 30 Hz camera frames, Supervisor::onFrame() with the simulated hand
// Sequence: home -joint-> pre-pick -vertical-> grasp height (0.3 s pause) -vertical-> pre-pick
// -joint-> home.
//
// The hand appears at random times near the path ahead of the robot: always once while the first
// joint segment is still accelerating and once during the vertical descent, plus random extra
// appearances (some of them too far away to intrude).
//
// Usage: sim_hand_slowdown [seed] [S_lim in J, default 0.05]     writes sim_hand.csv
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "poses.h"
#include "s_curve.h"
#include "segments.h"
#include "supervision.h"

namespace {
using Clock = std::chrono::steady_clock;
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kDt = 1e-3;
constexpr double kFramePeriod = 1.0 / 30.0;
constexpr double kGripperPause = 0.3;  // stands in for the grasp [s]

const scurve::JointLimits kJointLimits = scurve::frankaLimits(0.3, 0.3, 0.005);
constexpr motion::CartesianLimits kVerticalLimits{0.05, 0.3, 3.0};

enum SegmentIndex { kToPrePick, kDown, kUp, kToHome, kNumSegments };
const char* kSegmentNames[kNumSegments] = {"home->pre-pick", "down", "up", "pre-pick->home"};

// One hand appearance, timed from the start of a segment.
struct HandEvent {
  int segment;
  double t_on, t_off;  // relative to the segment start [s]
  Eigen::Vector3d position;
};

// Wall-clock start of each segment, written by the control thread.
std::atomic<double> g_segment_start[kNumSegments];

supervision::HandObservation handAt(const std::vector<HandEvent>& events, double now) {
  supervision::HandObservation obs;
  for (const HandEvent& e : events) {
    const double start = g_segment_start[e.segment].load();
    if (start >= 0.0 && now >= start + e.t_on && now < start + e.t_off) {
      obs.present = true;
      obs.position = e.position;
    }
  }
  return obs;
}

Eigen::Vector3d eeOf(const franka::Model& model, const franka::RobotState& state,
                     const scurve::Vec7& q) {
  const std::array<double, 16> T =
      model.pose(franka::Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
  return {T[12], T[13], T[14]};
}

struct LogRow {
  double t;
  int segment;
  scurve::Vec7 q;
  Eigen::Vector3d ee;
  supervision::HandObservation hand;
};
}  // namespace

int main(int argc, char** argv) {
  const unsigned seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1;
  supervision::Params params;
  if (argc > 2) params.s_lim = std::atof(argv[2]);

  try {
    franka::Robot robot(kRobotIp);
    const franka::Model model = robot.loadModel();
    franka::RobotState state = robot.readOnce();  // only for F_T_EE, EE_T_K and load

    // --- Nominal plan, to place the hand near where the robot will be.
    const scurve::JointSCurve to_pre(poses::kHome, poses::kPrePickJoints, kJointLimits);
    const Eigen::Vector3d p_pre = eeOf(model, state, poses::kPrePickJoints);
    const double z_grasp = poses::kTableZ + poses::kGraspHeight;
    const scurve::Profile1D vertical_nominal =
        scurve::planRestToRest(p_pre.z() - z_grasp, kVerticalLimits.v, kVerticalLimits.a,
                               kVerticalLimits.j);
    auto nominalEE = [&](int segment, double t) -> Eigen::Vector3d {
      switch (segment) {
        case kToPrePick: return eeOf(model, state, to_pre.at(t).q);
        case kDown: return p_pre - Eigen::Vector3d(0, 0, vertical_nominal.at(t).p);
        case kUp: return Eigen::Vector3d(p_pre.x(), p_pre.y(), z_grasp + vertical_nominal.at(t).p);
        default: return eeOf(model, state, to_pre.at(to_pre.duration() - t).q);
      }
    };
    const double nominal_duration[kNumSegments] = {to_pre.duration(), vertical_nominal.duration(),
                                                   vertical_nominal.duration(), to_pre.duration()};

    // --- Hand schedule.
    std::mt19937 rng(seed);
    auto uniform = [&](double a, double b) { return std::uniform_real_distribution<>(a, b)(rng); };
    auto randomOffset = [&](double max_radius) {
      Eigen::Vector3d d(uniform(-1, 1), uniform(-1, 1), uniform(-1, 1));
      return d.normalized() * uniform(0.0, max_radius);
    };
    std::vector<HandEvent> events;
    auto addEvent = [&](int seg, double t_on, double lead, double max_offset) {
      const Eigen::Vector3d p = nominalEE(seg, std::min(t_on + lead, nominal_duration[seg])) +
                                randomOffset(max_offset);
      events.push_back({seg, t_on, t_on + uniform(0.3, 1.2), p});
    };
    addEvent(kToPrePick, uniform(0.05, 0.2), uniform(0.2, 0.45), 0.03);  // while accelerating
    addEvent(kDown, uniform(0.3, 1.5), uniform(0.2, 0.45), 0.03);        // vertical descent
    for (int i = 0; i < 4; ++i) {
      const int seg = std::uniform_int_distribution<>(0, kNumSegments - 1)(rng);
      addEvent(seg, uniform(0.0, nominal_duration[seg]), uniform(0.1, 0.45), 0.08);
    }
    // One hand at a time: within a segment, end each appearance before the next one starts.
    std::sort(events.begin(), events.end(), [](const HandEvent& a, const HandEvent& b) {
      return a.segment != b.segment ? a.segment < b.segment : a.t_on < b.t_on;
    });
    for (size_t i = 0; i + 1 < events.size(); ++i) {
      if (events[i + 1].segment == events[i].segment) {
        events[i].t_off = std::min(events[i].t_off, events[i + 1].t_on - 0.05);
      }
    }
    events.erase(std::remove_if(events.begin(), events.end(),
                                [](const HandEvent& e) { return e.t_off <= e.t_on; }),
                 events.end());

    std::cout << "seed " << seed << ", S_lim " << params.s_lim << " J, radius "
              << params.radius * 100 << " cm, horizon " << params.horizon << " s\nHand schedule:\n";
    for (const HandEvent& e : events) {
      std::cout << "  " << kSegmentNames[e.segment] << " +" << e.t_on << "..+" << e.t_off
                << " s at (" << e.position.transpose() << ")\n";
    }

    // --- Threads.
    supervision::Shared shared;
    supervision::Supervisor supervisor(model, state, params, &shared);
    for (auto& s : g_segment_start) s = -1.0;
    std::atomic<bool> done{false};
    const Clock::time_point t0 = Clock::now();
    auto now = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };

    std::thread camera([&] {
      Clock::time_point next = Clock::now();
      while (!done) {
        supervisor.onFrame(handAt(events, now()), now());
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
      g_segment_start[seg] = now();
      for (double t = 0.0;; t += kDt) {
        const scurve::JointState ref = segment.step(t, q);
        log.push_back({now(), seg, q, eeOf(model, state, q), handAt(events, now())});
        if (segment.finished(t)) break;
        for (size_t i = 0; i < 7; ++i) q[i] += ref.dq[i] * kDt;
        waitTick();
      }
    };
    // Commanded EE pose, integrated from the vertical velocity commands as the robot does.
    std::array<double, 16> T{};
    auto runVertical = [&](int seg, double z_target) {
      motion::VerticalSegment segment(z_target, kVerticalLimits, &shared);
      g_segment_start[seg] = now();
      for (double t = 0.0;; t += kDt) {
        const double vz = segment.step(t, T, q);
        log.push_back({now(), seg, q, eeOf(model, state, q), handAt(events, now())});
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
