// Hand-aware speed supervision.
//
// The 1 kHz control loop publishes a Snapshot of the running segment every tick. The supervisor
// runs in its own thread, once per camera frame: it predicts the EE positions over the next
// horizon from the snapshot, and if one comes within `radius` of the hand it computes the
// highest speed at which every predicted point satisfies the energy limit
//   v_safe = sqrt(2 * S_lim / m_eff),  m_eff = 1 / (u^T [J M^-1 J^T]_pos u),
// u being the EE motion direction there. It sends that cap once and only looks again after the
// control loop reports the velocity change done and the reached speed was held for
// hold_after_slowdown. When the hand no longer intrudes, it sends the nominal speed to resume. The control loop applies a cap by re-planning from its own state.
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <franka/model.h>
#include <franka/robot_state.h>

#include "s_curve.h"
#include "seqlock.h"

namespace supervision {

struct Snapshot {
  uint32_t segment_id = 0;  // 0: no segment running
  bool vertical = false;    // vertical Cartesian segment, else joint S-curve
  double t = 0.0;           // time since the segment started [s]
  scurve::Profile1D profile;  // sigma (joint) or travelled distance (vertical); time 0 = t_replan
  double t_replan = 0.0;
  bool changing = false;      // velocity change of the last applied cap still running
  uint32_t applied_seq = 0;   // last command applied
  double cap = 0.0;           // active cap (nominal when not slowed)
  double nominal = 0.0;       // nominal sigma velocity [1/s] or vertical speed [m/s]
  scurve::Vec7 q{};           // current commanded joints
  scurve::Vec7 q_start{}, delta{};   // joint segment path: q = q_start + delta * sigma
  std::array<double, 3> p_start{};   // vertical segment: commanded EE position at start
  double direction = 0.0;            // vertical segment: +1 up, -1 down
};

struct Command {
  uint32_t segment_id = 0;  // only applied to this segment
  uint32_t seq = 0;
  double cap = 0.0;  // sigma velocity [1/s] (joint) or vertical speed [m/s]
};

// State shared between the control loop and the supervisor.
struct Shared {
  SeqLock<Snapshot> snapshot;
  SeqLock<Command> command;
  uint32_t next_segment_id = 1;  // only touched by the control thread
};

struct Params {
  double horizon = 0.5;   // prediction horizon [s]
  double sample_dt = 0.01;
  double radius = 0.05;   // intrusion distance between a predicted EE point and the hand [m]
  double s_lim = 0.05;    // transferable energy limit [J]
  double hold_after_slowdown = 0.3;  // no detection for this long once a slowdown is done [s]
};

struct HandObservation {
  bool present = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // base frame [m]
};

class Supervisor {
 public:
  // `state` provides the end-effector and load parameters for the model.
  Supervisor(const franka::Model& model, const franka::RobotState& state, const Params& params,
             Shared* shared);

  // Handles one camera frame. `now` is only used for the event log.
  void onFrame(const HandObservation& hand, double now);

  // Human-readable log of the decisions taken.
  std::vector<std::string> events() const;

  // Energy-limited EE speed at configuration q when moving in direction u (unit, base frame).
  double safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u) const;

  // Joint velocity that moves the EE vertically at 1 m/s (direction +1 up, -1 down) without
  // rotating, from the model at q.
  Eigen::Matrix<double, 7, 1> verticalJointRate(const scurve::Vec7& q, double direction) const;

 private:
  using Jacobian = Eigen::Matrix<double, 6, 7>;
  Jacobian jacobian(const scurve::Vec7& q) const;

  // Joint configurations at the predicted path parameters.
  std::vector<scurve::Vec7> predictJoints(const Snapshot& s, const std::vector<double>& path) const;

  // Returns the speed cap (in the snapshot's units) and whether the hand intrudes.
  double evaluate(const Snapshot& s, const Eigen::Vector3d& hand, bool* intrusion) const;
  void send(const Snapshot& s, double cap, double now, const std::string& why);

  const franka::Model& model_;
  franka::RobotState state_;  // F_T_EE, EE_T_K and load for the model
  Params params_;
  Shared* shared_;

  uint32_t seq_ = 0;
  uint32_t waiting_for_ = 0;  // command seq whose velocity change we wait for, 0: none
  bool holding_ = false;      // the command we wait for is a slowdown: hold afterwards
  uint32_t segment_ = 0;

  mutable std::mutex log_mutex_;
  std::vector<std::string> log_;
};

}  // namespace supervision
