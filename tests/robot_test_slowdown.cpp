// Robot test of the hand-aware slowdown with a virtual hand (no camera yet).
//
// Same pick sequence as robot_test_pick, with the supervisor running in its own thread at the
// camera rate (30 Hz) next to libfranka's 1 kHz control loop:
//   start -joint-> pre-pick, open, -vertical-> grasp height, grasp, -vertical-> pre-pick,
//   -joint-> start, release.   (start: home with joint 1 turned +90 deg)
// The virtual hand only exists in software: the robot slows down for it but nothing is there.
//
// Usage: robot_test_slowdown [SCHEDULE.csv | --random SEED] [S_lim in J, default 0.05]
//   e.g.  robot_test_slowdown ../config/hand_schedule.csv
//   writes robot_slowdown.csv (plot with scripts/plot_sim.py)
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "confirm.h"
#include "examples_common.h"
#include "motion.h"
#include "pick_sequence.h"
#include "poses.h"
#include "supervision.h"
#include "virtual_hand.h"

namespace {
using Clock = std::chrono::steady_clock;
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kFramePeriod = 1.0 / 30.0;
constexpr double kResetSpeedFactor = 0.2;
constexpr double kAtHomeTolerance = 1e-3;  // [rad]

constexpr double kGripperSpeed = 0.05;
constexpr double kGraspForce = 15.0;
constexpr double kGraspWidth = 0.048;  // test object
constexpr double kGraspEpsilon = 0.01;

struct FrameLog {
  double t;
  supervision::Snapshot snapshot;
  supervision::HandObservation hand;
};

// Runs the supervisor at the camera rate until stopped; joins on destruction.
class CameraThread {
 public:
  CameraThread(supervision::Supervisor* supervisor, virtual_hand::Hand* hand,
               supervision::Shared* shared, Clock::time_point t0)
      : thread_([=] {
          auto now = [t0] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
          Clock::time_point next = Clock::now();
          while (!done_) {
            const supervision::Snapshot s = shared->snapshot.load();
            const double t = now();
            const supervision::HandObservation obs = hand->observe(s, t);
            supervisor->onFrame(obs, t);
            log_.push_back({t, s, obs});
            next += std::chrono::microseconds(static_cast<int>(kFramePeriod * 1e6));
            std::this_thread::sleep_until(next);
          }
        }) {}
  ~CameraThread() { stop(); }
  void stop() {
    done_ = true;
    if (thread_.joinable()) thread_.join();
  }
  const std::vector<FrameLog>& log() const { return log_; }  // after stop()

 private:
  std::atomic<bool> done_{false};
  std::vector<FrameLog> log_;
  std::thread thread_;  // last: starts after the other members exist
};

void printJoint(const char* name, const motion::JointMotionResult& r) {
  std::cout << name << ": " << r.duration << " s, end error " << r.end_error * 1e3 << " mrad\n";
}

void printVertical(const char* name, double z_target, const motion::VerticalMotionResult& r) {
  std::cout << name << ": " << r.duration << " s, measured z error "
            << (r.z_end - z_target) * 1e3 << " mm, max xy deviation "
            << r.max_xy_deviation * 1e3 << " mm\n";
}

// Writes the camera-rate log in the format of sim_hand.csv. Positions come from the model at the
// logged joints; speeds are finite differences between frames of the same segment.
void writeLog(const std::vector<FrameLog>& log, const franka::Model& model,
              const franka::RobotState& state, const supervision::Supervisor& supervisor,
              const std::string& path) {
  std::ofstream csv(path);
  csv << std::setprecision(8) << "t,segment,x,y,z,speed,hand_present,hand_distance,v_safe\n";
  for (size_t k = 1; k < log.size(); ++k) {
    const FrameLog& r = log[k];
    const FrameLog& prev = log[k - 1];
    if (r.snapshot.segment_id == 0) continue;
    const Eigen::Vector3d ee = pick_sequence::eeOf(model, state, r.snapshot.q);
    const Eigen::Vector3d v = (ee - pick_sequence::eeOf(model, state, prev.snapshot.q)) /
                              (r.t - prev.t);
    const double speed = prev.snapshot.segment_id == r.snapshot.segment_id ? v.norm() : 0.0;
    double dist = -1.0, v_safe = -1.0;
    if (r.hand.present) {
      dist = (ee - r.hand.position).norm();
      if (speed > 1e-6) v_safe = supervisor.safeSpeed(r.snapshot.q, v / speed);
    }
    csv << r.t << "," << static_cast<int>(r.snapshot.segment_id) - 1 << "," << ee.x() << ","
        << ee.y() << "," << ee.z() << "," << speed << "," << r.hand.present << "," << dist << ","
        << v_safe << "\n";
  }
}
}  // namespace

int main(int argc, char** argv) {
  supervision::Params params;
  const int s_lim_arg = argc > 1 && std::string(argv[1]) == "--random" ? 3 : 2;
  if (argc > s_lim_arg) params.s_lim = std::atof(argv[s_lim_arg]);

  try {
    franka::Robot robot(kRobotIp);
    franka::Gripper gripper(kRobotIp);
    const franka::Model model = robot.loadModel();
    const franka::RobotState start = robot.readOnce();

    virtual_hand::Hand hand(
        pick_sequence::schedule(argc, argv, 1, pick_sequence::nominal(model, start)));
    std::cout << "S_lim " << params.s_lim << " J, radius " << params.radius * 100
              << " cm, horizon " << params.horizon << " s, hold after slowdown "
              << params.hold_after_slowdown << " s\n";

    if (gripper.readOnce().max_width < 0.01) {
      if (!confirm("gripper homing (fingers open fully, then close)")) return 0;
      gripper.homing();
    }
    const double max_width = gripper.readOnce().max_width;

    double start_offset = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      start_offset = std::max(start_offset, std::abs(start.q[i] - pick_sequence::kStart[i]));
    }
    if (start_offset > kAtHomeTolerance) {
      if (!confirm("reset to the start pose (home, joint 1 +90 deg) with MotionGenerator")) {
        return 0;
      }
      setDefaultBehavior(robot);
      MotionGenerator to_start(kResetSpeedFactor, pick_sequence::kStart);
      robot.control(to_start);
    }

    // Segment ids 1..4 must be the four segments below, in order.
    supervision::Shared shared;
    supervision::Supervisor supervisor(model, start, params, &shared);
    CameraThread camera(&supervisor, &hand, &shared, Clock::now());

    if (!confirm("S-curve start -> pre-pick (supervised)")) return 0;
    // Rejected while the user stop is pressed, so only set right before moving.
    setDefaultBehavior(robot);
    printJoint("start -> pre-pick",
               motion::moveJointSCurve(robot, poses::kPrePickJoints, pick_sequence::kJointLimits,
                                       &shared));
    const double z_pre_pick = robot.readOnce().O_T_EE_d[14];

    gripper.move(max_width, kGripperSpeed);
    if (!confirm("vertical down to 2 cm above the table (supervised)")) return 0;
    printVertical("down", pick_sequence::kZGrasp,
                  motion::moveVerticalTo(robot, pick_sequence::kZGrasp,
                                         pick_sequence::kVerticalLimits, &shared));

    const bool grasped =
        gripper.grasp(kGraspWidth, kGripperSpeed, kGraspForce, kGraspEpsilon, kGraspEpsilon);
    std::cout << "Grasp " << (grasped ? "succeeded" : "failed") << ", width "
              << gripper.readOnce().width << " m\n";

    if (!confirm("vertical up to pre-pick height (supervised)")) return 0;
    printVertical("up", z_pre_pick,
                  motion::moveVerticalTo(robot, z_pre_pick, pick_sequence::kVerticalLimits,
                                         &shared));

    if (!confirm("S-curve pre-pick -> start (supervised)")) return 0;
    printJoint("pre-pick -> start",
               motion::moveJointSCurve(robot, pick_sequence::kStart, pick_sequence::kJointLimits, &shared));

    camera.stop();
    std::cout << "\nSupervisor decisions:\n";
    for (const std::string& e : supervisor.events()) std::cout << "  " << e << "\n";
    if (supervisor.events().empty()) std::cout << "  (none)\n";
    writeLog(camera.log(), model, start, supervisor, "robot_slowdown.csv");
    std::cout << "Wrote robot_slowdown.csv (plot: python3 ../scripts/plot_sim.py "
                 "robot_slowdown.csv)\n";

    if (confirm("open gripper (release)")) {
      gripper.move(max_width, kGripperSpeed);
    }
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n"
              << "If the robot is in reflex mode, recover it in Desk before the next run.\n";
    return -1;
  }
  return 0;
}
