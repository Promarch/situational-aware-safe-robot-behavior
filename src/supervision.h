// Hand-aware speed supervision of the whole robot.
//
// The 1 kHz control loop publishes a Snapshot of the running motion every tick. The supervisor
// runs in its own thread, once per camera frame. From the snapshot it predicts the joints over the
// next `horizon` and places the robot's capsules (robot_body.h: links 2-7, hand, fingers, the held
// object) at every predicted configuration. A hand is *exposed* to a capsule when, at some sample,
// the capsule's surface comes closer to the hand's sphere center than the sphere radius plus
// `radius_margin`. If a hand is exposed to some capsule (or a hand is seen without a position, which
// exposes every capsule), the supervisor computes the highest speed at which every possible contact
// point on the exposed capsules' axes satisfies the energy limit over the whole horizon:
//   v_safe = sqrt(2 * S_lim / m),  m_eff = 1 / (u^T [J_p M^-1 J_p^T]_pos u),
// with J_p the Jacobian of the contact point p and u its motion direction. m depends on the contact
// (Contact) and the capsule's group (see limitMassAt). It sends that cap once, and looks again only
// after the control loop reports the velocity change done and the reached speed was held for
// `hold_after_slowdown`. When no hand is exposed anymore, it sends the nominal speed to resume.
// A resume is not waited for: every frame is still checked, and a slowdown replaces it.
// The control loop applies a cap by re-planning from its own state (see motion.cpp).
// docs/collision_detection.md describes the scenarios.
//
// The *reference point* (the EE, or the held object's far end) is tested itself only without
// `check_links` (the test before the capsules) or without `franka_hand`. It is also used for the
// logs and the overlay's predicted path.
//
// "Motion" here is one supervised motion call (moveJointSCurve or moveVerticalTo), identified by
// motion_id; not to be confused with the constant-jerk segments of a Profile1D.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <franka/model.h>
#include <franka/robot_state.h>

#include "mass_table.h"
#include "robot_body.h"
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
  // the robot at rest): the object is then part of the robot's capsules, and the reference point
  // is its far end.
  std::atomic<bool> holding_object{false};
};

struct Params {
  double horizon = 0.5;   // prediction horizon [s]
  double sample_dt = 0.01;
  // A hand is exposed to a capsule when, at a predicted sample, the capsule's surface is closer to
  // the hand's sphere center than the sphere radius sent by the detector plus this margin [m].
  double radius_margin = 0.20;
  // Held object, in the EE frame: a capsule from the EE origin to `object_tip` (far end; x, y, z
  // along the EE's own axes) with radius `object_radius` [m]. The far end is also the reference
  // point while the object is held, else the EE itself.
  Eigen::Vector3d object_tip = Eigen::Vector3d::Zero();
  double object_radius = 0.015;
  // Robot geometry (robot_body::BodyConfig). check_links = false: only the reference point is
  // tested, exactly as before the capsules (comparison runs only: links 2-7 and the gripper are then
  // untested). franka_hand = false: another tool, no hand capsules; the reference point is tested
  // instead. link_safety_distance: added to every capsule radius, for robot parts the capsules do
  // not cover (cables, covers, fit) [m]; radius_margin is for the hand.
  bool check_links = true;
  bool franka_hand = true;
  double link_safety_distance = 0.0;
  double s_lim = 0.01;    // transferable energy limit [J]
  double hold_after_slowdown = 0.3;  // no detection for this long once a slowdown is done [s]
  // A hand trapped against the table cannot move away: during a downward vertical motion, when an
  // exposed hand's center is at most `constrained_band` above the table, the energy limit uses the
  // reflected mass tabulated in `constrained_mass` (measured at the EE) instead of the model's
  // effective mass. Other motions are always free contacts.
  double table_z = 0.0;            // table surface in the base frame [m]
  double constrained_band = 0.05;  // [m]
  const MassTable* constrained_mass = nullptr;  // must outlive the supervisor
};

// How a hand can be hit, which decides the mass used for the energy limit.
enum class Contact {
  kFree,         // the hand can move away: the model's effective mass
  kConstrained,  // the hand is near the table (downward motion): the tabulated reflected mass
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
  // is unknown, so every capsule counts as exposed.
  bool unlocated = false;
};

// How the supervisor sees one hand in the current frame.
enum class HandStatus : uint8_t {
  kClear,        // no capsule comes within its intrusion distance over the horizon
  kIntruding,    // exposed to at least one capsule, free contact
  kConstrained,  // exposed, near the table during a downward motion
};

// One hand against the robot over the horizon.
struct HandAssessment {
  HandStatus status = HandStatus::kClear;
  uint8_t capsule = robot_body::kNoCapsule;  // nearest capsule over the horizon
  // Distance from the hand's intrusion boundary (sphere radius + radius_margin) to that capsule's
  // surface at its closest; < 0: exposed [m].
  double clearance = 0.0;
  Eigen::Vector3d closest = Eigen::Vector3d::Zero();  // nearest robot surface point, base frame
  uint16_t exposed = 0;  // bit i: exposed to capsule id i
};

// What the supervisor saw in one camera frame, for display only (the detector's video overlay).
struct Overlay {
  bool valid = false;           // the control loop has published a state
  bool moving = false;          // a supervised motion runs: `predicted` and `hands` status are set
  bool holding_object = false;  // reference point is the object's far end, else the EE
  bool slowed = false;          // the last cap sent for this motion is a slowdown
  Contact contact = Contact::kFree;  // contact at the point that set that slowdown
  double cap_fraction = 1.0;         // that cap / nominal speed
  // Capsule and contact point (base frame) that set that slowdown; kNoCapsule if not slowed.
  uint8_t limiting_capsule = robot_body::kNoCapsule;
  Eigen::Vector3d limiting_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference = Eigen::Vector3d::Zero();  // current reference point, base frame
  std::vector<Eigen::Vector3d> predicted;  // reference point every sample_dt over the horizon
  std::vector<robot_body::Capsule> capsules;  // at the current joints
  HandObservation hands;
  std::array<HandAssessment, kMaxHands> assessment{};
};

// One command sent by the supervisor.
struct Decision {
  uint32_t motion_id;
  double t_motion;      // time since the motion started [s]
  bool slow_down;       // false: resume to nominal
  double cap_fraction;  // cap / nominal speed
  Contact contact;      // contact at the point that set the cap (kFree for a resume)
  uint8_t capsule;      // capsule that set the cap (kNoCapsule for a resume)
};

class Supervisor {
 public:
  // `state` provides the end-effector and load parameters for the model.
  Supervisor(const franka::Model& model, const franka::RobotState& state, const Params& params,
             Shared* shared);

  // Handles one camera frame.
  void onFrame(const HandObservation& hands);

  // Called at the end of every onFrame with what the supervisor saw (display only).
  void setOverlaySink(std::function<void(const Overlay&)> sink) { overlay_sink_ = std::move(sink); }

  const robot_body::RobotBody& body() const { return body_; }

  // Effective (reflected) mass of the robot at the reference point in direction u (unit, base
  // frame): m_eff = 1 / (u^T [J M^-1 J^T]_pos u), J and M from libfranka's model at q [kg].
  double effectiveMass(const scurve::Vec7& q, const Eigen::Vector3d& u, bool at_object_tip) const;

  // Mass used for the energy limit at the reference point for the given contact: the effective
  // mass in direction u, and/or the tabulated mass at the EE position.
  double limitMass(const scurve::Vec7& q, const Eigen::Vector3d& u, bool at_object_tip,
                   Contact contact) const;

  // Energy-limited speed of the reference point (the object's far end if `at_object_tip`, else
  // the EE) at configuration q when moving in direction u (unit, base frame):
  // sqrt(2 S_lim / limitMass).
  double safeSpeed(const scurve::Vec7& q, const Eigen::Vector3d& u, bool at_object_tip,
                   Contact contact) const;

  // Mass for the energy limit at a contact point on capsule `capsule`, given its effective mass
  // m_eff there, at configuration q:
  //   free: m_eff;
  //   constrained: tool group the table at the EE, arm group max(m_eff, the table's largest mass);
  //   unknown: max(m_eff, the constrained mass above).
  // The table was measured at the EE, so for links 2-6 there is no data: the largest value is the
  // conservative choice.
  double limitMassAt(const scurve::Vec7& q, double m_eff, uint8_t capsule, Contact contact) const;

  // Whether a hand counts as trapped against the table.
  bool nearTable(const HandSphere& hand) const {
    return hand.center.z() - params_.table_z <= params_.constrained_band;
  }

  // The hand's clearance to the robot's capsules: the distance from its intrusion boundary to the
  // nearest capsule surface (< 0: exposed), the capsule and that surface point.
  robot_body::Nearest clearance(const std::vector<robot_body::Capsule>& capsules,
                                const HandSphere& hand) const;

  // Flange-to-reference-point transform for libfranka's model, in place of F_T_EE.
  const std::array<double, 16>& flangeToReference(bool at_object_tip) const {
    return at_object_tip ? F_T_tip_ : state_.F_T_EE;
  }

  // Commands sent so far. Read only after the supervisor thread has stopped.
  const std::vector<Decision>& decisions() const { return decisions_; }
  // Longest onFrame so far [s]. Read only after the supervisor thread has stopped.
  double maxFrameTime() const { return max_frame_time_; }

 private:
  using Jacobian = Eigen::Matrix<double, 6, 7>;
  using Matrix7 = Eigen::Matrix<double, 7, 7>;

  // One predicted configuration with the robot's placement there.
  struct Sample {
    scurve::Vec7 q;
    robot_body::Frames frames;
    std::vector<robot_body::Capsule> capsules;
  };
  // The supervisor's view of one frame: predicted samples and every hand against them.
  struct View {
    std::vector<Sample> samples;
    std::array<HandAssessment, kMaxHands> hands{};
  };
  struct Evaluation {
    double cap = 0.0;
    bool intrusion = false;
    Contact contact = Contact::kFree;
    uint8_t capsule = robot_body::kNoCapsule;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
  };

  Jacobian jacobian(const scurve::Vec7& q, bool at_object_tip) const;
  Matrix7 massInverse(const scurve::Vec7& q) const;
  // Joint velocity moving the EE vertically at 1 m/s (+1 up, -1 down) without rotating.
  Eigen::Matrix<double, 7, 1> verticalJointRate(const scurve::Vec7& q, double direction) const;
  // Tabulated reflected mass at the EE position of configuration q.
  double tabulatedMass(const scurve::Vec7& q) const;
  // Joints over the horizon, every sample_dt, from the snapshot's motion.
  std::vector<scurve::Vec7> predict(const Snapshot& s) const;
  // Reference point (base frame) at configuration q.
  Eigen::Vector3d referencePoint(const scurve::Vec7& q, bool at_object_tip) const;
  // Predicted samples and every hand's assessment against them.
  View view(const Snapshot& s, bool holding, const HandObservation& hands) const;
  // Speed cap in the snapshot's units, with what set it.
  Evaluation evaluate(const Snapshot& s, const View& v, const HandObservation& hands) const;
  void decide(const Snapshot& s, const View& v, const HandObservation& hands);
  void send(const Snapshot& s, const Evaluation& e);
  Overlay overlay(const Snapshot& s, bool holding, const View& v,
                  const HandObservation& hands) const;

  const franka::Model& model_;
  franka::RobotState state_;  // F_T_EE, EE_T_K and load for the model
  std::array<double, 16> F_T_tip_;  // F_T_EE * translation(object_tip)
  Params params_;
  Shared* shared_;
  robot_body::RobotBody body_;

  uint32_t seq_ = 0;
  uint32_t waiting_for_ = 0;  // slowdown seq whose velocity change and hold we wait for, 0: none
  bool holding_ = false;      // the last command sent is a slowdown
  uint32_t motion_ = 0;
  // Last command for the current motion (nominal until the first command). Compared against
  // instead of the snapshot's cap, which lags until the control loop applies a command.
  Evaluation target_;
  std::vector<Decision> decisions_;
  std::function<void(const Overlay&)> overlay_sink_;
  double max_frame_time_ = 0.0;
};

}  // namespace supervision
