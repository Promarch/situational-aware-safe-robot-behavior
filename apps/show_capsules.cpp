// Checks the supervisor's robot capsules (robot_body.h) against the real robot. The robot does NOT
// move: only its state and model are read.
//
//  1. Prints the capsules at the current joints.
//  2. Checks the velocity of every capsule end as the energy limit computes it: libfranka's
//     model.zeroJacobian of the capsule's frame, shifted from the frame origin to the point
//     (RobotBody::pointJacobian: J_v - [p - o]x J_w), against central differences of model.pose
//     (the point moved by +-1e-6 rad per joint), and prints the largest deviation per capsule. It
//     also reports whether libfranka's Jacobian of joint frame N contains joint N's own rotation,
//     which the shift needs. Exit code 2 if the check fails.
//  3. --poses: prints the capsules along the task's joint motions (start -> pre-pick ->
//     pre-place -> start), for placing replayed hands (config/hands_replay*.csv), and exits.
//  4. Otherwise it streams the capsules at the current joints as a robot overlay (overlay_udp.h,
//     the robot at rest) at 30 Hz until Ctrl+C. Guide the robot by hand and check in the detector's
//     video that the capsules cover the links.
//
// Usage: show_capsules [--holding] [--no-links] [--no-hand] [--safety M] [--poses] [--port 5006]
//   --holding   include the held screwdriver (as in pick_place)
//   --no-links, --no-hand, --safety M   as supervision::Params check_links / franka_hand /
//               link_safety_distance
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "overlay_udp.h"
#include "poses.h"
#include "robot_body.h"
#include "supervision.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kFiniteStep = 1e-6;   // [rad]
constexpr double kTolerance = 1e-5;    // Jacobian deviation counted as a disagreement [m/rad]

using Linear = robot_body::LinearJacobian;

std::atomic<bool> stop{false};
void onSignal(int) { stop = true; }

void printCapsules(const std::vector<robot_body::Capsule>& capsules) {
  for (const robot_body::Capsule& c : capsules) {
    std::cout << "  " << std::setw(9) << robot_body::capsuleName(c.id) << "  a "
              << c.a.transpose() << "  b " << c.b.transpose() << "  r " << c.radius << "\n";
  }
}

class Checker {
 public:
  explicit Checker(const robot_body::RobotBody& body) : body_(body) {}

  // Linear Jacobian of point p (fixed to frame index `frame`) by central differences of the pose.
  Linear finiteDifferences(int frame, const scurve::Vec7& q, const Eigen::Vector3d& p) const {
    const Eigen::Vector3d p_local = body_.frames(q)[frame].inverse() * p;
    Linear J;
    for (int i = 0; i < 7; ++i) {
      scurve::Vec7 qp = q, qm = q;
      qp[i] += kFiniteStep;
      qm[i] -= kFiniteStep;
      J.col(i) = (body_.frames(qp)[frame] * p_local - body_.frames(qm)[frame] * p_local) /
                 (2.0 * kFiniteStep);
    }
    return J;
  }

  // Returns false if the supervisor's point Jacobian disagrees with the reference.
  bool run(const scurve::Vec7& q, bool holding) const {
    const robot_body::Frames frames = body_.frames(q);
    std::cout << std::scientific << std::setprecision(1)
              << "Capsule end velocities (libfranka Jacobian shifted to the point): largest\n"
              << "deviation from central differences of model.pose [m/rad]\n";
    double worst = 0.0;
    for (const robot_body::Capsule& c : body_.capsules(frames, holding)) {
      double deviation = 0.0;
      for (const Eigen::Vector3d& p : {c.a, c.b}) {
        const Linear J = body_.pointJacobian(q, frames, c.frame, p);
        deviation = std::max(deviation,
                             (J - finiteDifferences(c.frame, q, p)).cwiseAbs().maxCoeff());
      }
      std::cout << "  " << std::setw(9) << robot_body::capsuleName(c.id) << "  " << deviation
                << "\n";
      worst = std::max(worst, deviation);
    }

    // The shift J_v - [r]x J_w needs the frame's full angular velocity: libfranka's Jacobian of
    // joint frame N must contain joint N's own rotation (angular column N = z_N).
    std::cout << "libfranka zeroJacobian(kJointN), angular column N vs. the joint axis z_N:\n";
    for (int n = 0; n < 7; ++n) {
      const Eigen::Vector3d column = body_.frameJacobian(q, n).block<3, 1>(3, n);
      const Eigen::Vector3d z = frames[n].linear().col(2);
      std::cout << "  kJoint" << n + 1 << ": "
                << ((column - z).norm() < kTolerance ? "contains joint " + std::to_string(n + 1)
                    : column.norm() < kTolerance     ? "joint " + std::to_string(n + 1) + " MISSING"
                                                     : "differs")
                << "\n";
    }
    const bool ok = worst < kTolerance;
    std::cout << "Result: point Jacobian " << (ok ? "OK" : "WRONG") << " (tolerance " << kTolerance
              << ")\n"
              << std::fixed << std::setprecision(3);
    return ok;
  }

 private:
  const robot_body::RobotBody& body_;
};
}  // namespace

int main(int argc, char** argv) {
  bool holding = false, poses_only = false;
  uint16_t port = 5006;
  robot_body::BodyConfig config;
  config.object_tip = Eigen::Vector3d(0.15, 0.0, 0.0);  // the screwdriver, as in pick_place
  config.object_radius = 0.015;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--holding") == 0) {
      holding = true;
    } else if (std::strcmp(argv[i], "--no-links") == 0) {
      config.check_links = false;
    } else if (std::strcmp(argv[i], "--no-hand") == 0) {
      config.franka_hand = false;
    } else if (std::strcmp(argv[i], "--safety") == 0 && i + 1 < argc) {
      config.safety_distance = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--poses") == 0) {
      poses_only = true;
    } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else {
      std::cerr << "Usage: show_capsules [--holding] [--no-links] [--no-hand] [--safety M] "
                   "[--poses] [--port 5006]\n";
      return 1;
    }
  }

  bool jacobian_ok = false;
  try {
    franka::Robot robot(kRobotIp);
    const franka::Model model = robot.loadModel();
    franka::RobotState state = robot.readOnce();
    const robot_body::RobotBody body(model, state, config);
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "Capsules at the current joints (base frame [m]):\n";
    printCapsules(body.capsules(state.q, holding));
    jacobian_ok = Checker(body).run(state.q, holding);
    if (!jacobian_ok) {
      std::cerr << "The supervisor's point Jacobian disagrees with the model: do not use it.\n";
    }

    if (poses_only) {
      struct Leg {
        const char* name;
        scurve::Vec7 from, to;
      };
      for (const Leg& leg : {Leg{"start -> pre-pick", poses::kStart, poses::kPrePickJoints},
                             Leg{"pre-pick -> pre-place", poses::kPrePickJoints,
                                 poses::kPrePlaceJoints},
                             Leg{"pre-place -> start", poses::kPrePlaceJoints, poses::kStart}}) {
        for (double sigma : {0.0, 0.25, 0.5, 0.75, 1.0}) {
          scurve::Vec7 q;
          for (int i = 0; i < 7; ++i) q[i] = leg.from[i] + sigma * (leg.to[i] - leg.from[i]);
          std::cout << "\n" << leg.name << ", sigma " << sigma << ":\n";
          printCapsules(body.capsules(q, holding));
        }
      }
      return jacobian_ok ? 0 : 2;
    }

    OverlaySender overlay("127.0.0.1", port);
    std::signal(SIGINT, onSignal);
    std::cout << "Streaming the capsules to UDP 127.0.0.1:" << port << " at 30 Hz, Ctrl+C to stop\n";
    while (!stop) {
      state = robot.readOnce();
      supervision::Overlay o;
      o.valid = true;
      o.holding_object = holding;
      const robot_body::Frames frames = body.frames(state.q);
      o.reference = frames[robot_body::kEndEffector] *
                    (holding ? config.object_tip : Eigen::Vector3d::Zero());
      o.capsules = body.capsules(frames, holding);
      overlay.send(o);
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n";
    return -1;
  }
  return jacobian_ok ? 0 : 2;
}
