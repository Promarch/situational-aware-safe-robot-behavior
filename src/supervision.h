// Hand-aware speed supervision.
//
// The 1 kHz control loop publishes a Snapshot of the running motion every tick. The supervisor
// runs in its own thread, once per camera frame: it predicts the EE positions over the next
// `horizon` from the snapshot, and if one comes within any hand's sphere radius plus
// `radius_margin` of that hand's center (or a hand is seen without a position) it computes the
// highest speed at which every predicted point satisfies the energy limit
//   v_safe = sqrt(2 * S_lim / m_eff),  m_eff = 1 / (u^T [J M^-1 J^T]_pos u),
// u being the EE motion direction there. It sends that cap once and looks again only after the
// control loop reports the velocity change done and the reached speed was held for
// `hold_after_slowdown`. When the hand no longer intrudes, it sends the nominal speed to resume;
// a resume is not waited for: every frame is still checked, and a slowdown replaces it.
// The control loop applies a cap by re-planning from its own state (see motion.cpp).
//
// "Motion" here is one supervised motion call (moveJointSCurve or moveVerticalTo), identified by
// motion_id; not to be confused with the constant-jerk segments of a Profile1D.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <franka/model.h>
#include <franka/robot_state.h>

#include "mass_table.h"
#include "s_curve.h"
#include "seqlock.h"

namespace supervision {

struct Snapshot {
  uint32_t motion_id = 0;   // 0: no motion running
  bool vertical = false;    // vertical Cartesian motion, else joint S-curve
  double t = 0.0;           // time since the motion started [s]
  scurve::Profile1D profile;  // sigma (joint) or travelled distance (vertical); time 0 = t_replan
  double t_replan = 0.0;
  bool changing = false;      // velocity change of the last applied cap still running
  uint32_t applied_seq = 0;   // last command applied
  double cap = 0.0;           // active cap (nominal when not slowed)
  double nominal = 0.0;       // nominal sigma velocity [1/s] or vertical speed [m/s]
  scurve::Vec7 q{};           // current joints
  scurve::Vec7 q_start{}, delta{};   // joint motion path: q = q_start + delta * sigma
  double direction = 0.0;            // vertical motion: +1 up, -1 down
};

struct Command {
  uint32_t motion_id = 0;  // only applied to this motion
  uint32_t seq = 0;
  double cap = 0.0;  // sigma velocity [1/s] (joint) or vertical speed [m/s]
};

// State shared between the control loop and the supervisor.
struct Shared {
  SeqLock<Snapshot> snapshot;
  SeqLock<Command> command;
  uint32_t next_motion_id = 1;  // only touched by the control thread
  // Set by the main program while the gripper holds the object (between grasp and release, with
  // the robot at rest): the supervisor then uses the object's far end as the reference point.
  std::atomic<bool> holding_object{false};
};

struct Params {
  double horizon = 0.5;   // prediction horizon [s]
  double sample_dt = 0.01;
  // A hand intrudes when a predicted reference point is closer to its sphere center than the
  // sphere radius sent by the detector plus this margin [m].
  double radius_margin = 0.20;
  // While an object is held, the reference point for the intrusion test and the energy limit is
  // its far end, at this position in the EE frame (fingertip center; x, y, z along the EE's own
  // axes) [m]. Otherwise it is the EE itself.
  Eigen::Vector3d object_tip = Eigen::Vector3d::Zero();
  double s_lim = 0.01;    // transferable energy limit [J]
  double hold_after_slowdown = 0.3;  // no detection for this long once a slowdown is done [s]
  // A hand trapped against the table cannot move away: during a downward vertical motion, when an
  // intruding hand's center is at most `constrained_band` above the table, the energy limit uses
  // the reflected mass tabulated in `constrained_mass` at the EE position instead of the model's
  // effective mass. Other motions are always free contacts.
  double table_z = 0.0;            // table surface in the base frame [m]
  double constrained_band = 0.05;  // [m]
  const MassTable* constrained_mass = nullptr;  // must outlive the supervisor
};

// How a hand that intrudes can be hit, which decides the mass used for the energy limit.
enum class Contact {
  kFree,         // the hand can move away: the model's effective mass
  kConstrained,  // the hand is near the table: the tabulated reflected mass
  kUnknown,      // the hand's position is unknown (downward motion): the larger of the two
};

// Hands of one camera frame, as the detector sends them (hand_udp protocol, udp-implementation
// branch): each hand is a sphere in the robot base frame.
constexpr int kMaxHands = 6;  // hand_udp::kMaxHands

struct HandSphere {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // [m]
  double radius = 0.0;                               // [m]
};

struct HandObservation {
  int n_hands = 0;
  std::array<HandSphere, kMaxHands> hands{};
  // A hand was seen but has no usable 3D position (protocol flag HAND_UNLOCATED): its position
  // is unknown, so it is treated as intruding.
  bool unlocated = false;
};

// One command sent by the supervisor.
struct Decision {
  uint32_t motion_id;
  double t_motion;      // time since the motion started [s]
  bool slow_down;       // false: resume to nominal
  double cap_fraction;  // cap / nominal speed
  Contact contact;      // contact the cap was computed for (kFree for a resume)
};

class Supervisor {
 public:
  // `state` provides the end-effector and load parameters for the model.
  Supervisor(const franka::Model& model, const franka::RobotState& state, const Params& params,
             Shared* shared);

  // Handles one camera frame.
  void onFrame(const HandObservation& hands);

  // Effective (reflected) mass of the robot at the reference point in direction u (unit, base
  // frame): m_eff = 1 / (u^T [J M^-1 J^T]_pos u), J and M from libfranka's model at q [kg].
  double effectiveMass(const scurve::Vec7& q, const Eigen::Vector3d& u, bool at_object_tip) const;

  // Mass used for the energy limit at configuration q for the given contact: the effective mass
  // at the reference point in direction u, and/or the tabulated mass at the EE position.
  double limitMass(const scurve::Vec7& q, const Eigen::Vector3d& u, bool at_object_tip,
                   Contact contact) const;

  // Energy-limited speed of the reference point (the object's far end if `at_object_tip`, else
  // the EE) at configuration q when moving in direction u (unit, base frame):
  // sqrt(2 S_lim / limitMass).
  double safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u, bool at_object_tip,
                   Contact contact) const;

  // Whether a hand counts as trapped against the table.
  bool nearTable(const HandSphere& hand) const {
    return hand.center.z() - params_.table_z <= params_.constrained_band;
  }

  // Flange-to-reference-point transform for libfranka's model, in place of F_T_EE.
  const std::array<double, 16>& flangeToReference(bool at_object_tip) const {
    return at_object_tip ? F_T_tip_ : state_.F_T_EE;
  }

  // Commands sent so far. Read only after the supervisor thread has stopped.
  const std::vector<Decision>& decisions() const { return decisions_; }

 private:
  using Jacobian = Eigen::Matrix<double, 6, 7>;
  Jacobian jacobian(const scurve::Vec7& q, bool at_object_tip) const;
  // Joint velocity moving the EE vertically at 1 m/s (+1 up, -1 down) without rotating.
  Eigen::Matrix<double, 7, 1> verticalJointRate(const scurve::Vec7& q, double direction) const;
  // Tabulated reflected mass at the EE position of configuration q.
  double tabulatedMass(const scurve::Vec7& q) const;
  // Speed cap in the snapshot's units; sets *intrusion and the *contact it was computed for.
  double evaluate(const Snapshot& s, const HandObservation& hands, bool* intrusion,
                  Contact* contact) const;
  void send(const Snapshot& s, double cap, Contact contact);

  const franka::Model& model_;
  franka::RobotState state_;  // F_T_EE, EE_T_K and load for the model
  std::array<double, 16> F_T_tip_;  // F_T_EE * translation(object_tip)
  Params params_;
  Shared* shared_;

  uint32_t seq_ = 0;
  uint32_t waiting_for_ = 0;  // slowdown seq whose velocity change and hold we wait for, 0: none
  bool holding_ = false;      // the last command sent is a slowdown
  uint32_t motion_ = 0;
  // Cap last commanded for the current motion (nominal until the first command). Compared
  // against instead of the snapshot's cap, which lags until the control loop applies a command.
  double target_cap_ = 0.0;
  std::vector<Decision> decisions_;
};

}  // namespace supervision
