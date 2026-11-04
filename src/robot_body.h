// The robot's volume as capsules (a segment plus a radius) attached to libfranka's link frames, for
// the supervisor's whole-robot intrusion test and energy limit.
//
// Frames are libfranka's Frame::kJoint1..kJoint7 (modified DH, the franka_description URDF link
// frames panda_link1..7: joint i turns about the z axis of frame i, and frame i moves with link i),
// Frame::kFlange and Frame::kEndEffector (F_T_EE from Desk). Each capsule is fixed in one of them.
//
// Capsules: franka_description's coarse self-collision capsules (franka_arm.xacro /
// franka_hand.xacro, `self_collision_geometries`: center, axis, length, radius in the link frame).
// Left out: link 0 (the base does not move) and link 1 (it only spins about its own axis, so its
// capsule never moves towards anything). The Franka Hand's two capsules are given in the flange
// frame (the hand is mounted -45 deg about the flange z axis), so they do not depend on the end
// effector configured in Desk. Check them on the robot with show_capsules and the detector's video.
//
// Options (BodyConfig):
//   check_links = false: only the reference point (radius 0): the EE, or the held object's far end
//     while it is held. Exactly the test before the capsules, for comparison runs.
//   franka_hand = false: no hand capsules (another tool). The reference point is then added, so the
//     tool center is still tested; add capsules for the tool in fixedCapsules().
//   safety_distance: added to every capsule radius (not to the reference point) [m].
//
// Velocities of points on the robot (for the energy limit) come from libfranka's Jacobian of the
// capsule's frame, shifted to the point (shiftToPoint). show_capsules checks them on the robot
// against finite differences of model.pose.
//
// Everything here except RobotBody is pure geometry and needs no robot.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <franka/model.h>
#include <franka/robot_state.h>

#include "s_curve.h"

namespace robot_body {

// Frame indices: 0..6 joint frames 1..7, 7 the flange, 8 the end effector.
constexpr int kNumFrames = 9;
constexpr int kFlange = 7;
constexpr int kEndEffector = 8;
using Frames = std::array<Eigen::Isometry3d, kNumFrames>;  // base frame
using Jacobian = Eigen::Matrix<double, 6, 7>;        // linear rows 0-2, angular 3-5
using LinearJacobian = Eigen::Matrix<double, 3, 7>;

// libfranka's frame for a frame index.
franka::Frame frankaFrame(int index);

// Capsule ids (also on the overlay wire format, see overlay_udp.h).
enum CapsuleId : uint8_t {
  kLink2, kLink3, kLink4, kLink5, kLink5b, kLink6, kLink7, kLink7b,
  kHand, kFingers,
  kObject,     // the held object, only while holding
  kReference,  // a point (radius 0): only without check_links or without franka_hand
  kNumCapsules,
};
constexpr uint8_t kNoCapsule = 255;

const char* capsuleName(uint8_t id);  // "link4", ..., "none" for kNoCapsule
// The tool group (link 7, hand, fingers, held object, reference point) is within ~20 cm of the
// EE, where the constrained-collision mass table was measured. The arm group (links 2-6) is not.
inline bool isTool(uint8_t id) { return id >= kLink7 && id < kNumCapsules; }

struct CapsuleSpec {
  uint8_t id;
  int frame;               // frame index
  Eigen::Vector3d a, b;    // segment ends in that frame [m]
  double radius;           // [m]
};
// The fixed link capsules (link2 .. link7b) and the Franka Hand's (hand, fingers), in id order.
const std::array<CapsuleSpec, kObject>& fixedCapsules();

struct BodyConfig {
  // Held object, in the EE frame: a capsule from the EE origin to `object_tip` with radius
  // `object_radius`. Its far end is the reference point while it is held, else the EE origin.
  Eigen::Vector3d object_tip = Eigen::Vector3d::Zero();
  double object_radius = 0.0;
  bool check_links = true;
  bool franka_hand = true;
  double safety_distance = 0.0;
};

struct Capsule {
  Eigen::Vector3d a, b;  // segment ends, base frame [m]
  double radius = 0.0;
  uint8_t id = kNoCapsule;
  int frame = 0;  // frame index it is fixed in
};

// Capsules in the base frame for the given frames and options; the object only while `holding`.
std::vector<Capsule> placeCapsules(const Frames& frames, bool holding, const BodyConfig& config);

// Segment parameter t in [0, 1] of the point of a-b closest to p.
double closestOnSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                        const Eigen::Vector3d& p);
inline Eigen::Vector3d pointOn(const Capsule& c, double t) { return c.a + t * (c.b - c.a); }

// The part [t0, t1] of segment a-b inside the sphere (center, R). False if none.
bool segmentChord(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                  const Eigen::Vector3d& center, double R, double* t0, double* t1);

// Point on the capsule's surface at axis parameter t, on the side facing `toward` (any side
// perpendicular to the axis if `toward` lies on the axis). For display: the energy limit uses axis
// points, which approach a hand exactly as fast as the surface point facing it.
Eigen::Vector3d surfacePoint(const Capsule& c, double t, const Eigen::Vector3d& toward);

// Capsule nearest to a point: its index in the list, the distance from the point to its surface
// (negative inside) and the nearest surface point.
struct Nearest {
  int index = -1;
  double distance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
};
Nearest nearestCapsule(const std::vector<Capsule>& capsules, const Eigen::Vector3d& p);

// Linear Jacobian of a point rigidly attached to a frame, from the frame's Jacobian (base frame)
// and the point's offset r = p - frame origin (base frame): v_p = v_o + w x r = J_v dq - [r]x J_w dq.
LinearJacobian shiftToPoint(const Jacobian& frame_jacobian, const Eigen::Vector3d& r);

// The libfranka-dependent part: frames and Jacobians at a configuration.
class RobotBody {
 public:
  // `state` provides F_T_EE and EE_T_K.
  RobotBody(const franka::Model& model, const franka::RobotState& state, const BodyConfig& config);

  Frames frames(const scurve::Vec7& q) const;
  std::vector<Capsule> capsules(const Frames& frames, bool holding) const {
    return placeCapsules(frames, holding, config_);
  }
  std::vector<Capsule> capsules(const scurve::Vec7& q, bool holding) const {
    return capsules(frames(q), holding);
  }
  // libfranka's zeroJacobian of frame index `frame` (base frame, at the frame's origin).
  Jacobian frameJacobian(const scurve::Vec7& q, int frame) const;
  // Linear Jacobian of point p (base frame) fixed to frame index `frame`, with `frames` at q.
  LinearJacobian pointJacobian(const scurve::Vec7& q, const Frames& frames, int frame,
                               const Eigen::Vector3d& p) const {
    return shiftToPoint(frameJacobian(q, frame), p - frames[frame].translation());
  }
  const BodyConfig& config() const { return config_; }

 private:
  const franka::Model& model_;
  std::array<double, 16> F_T_EE_, EE_T_K_;
  BodyConfig config_;
};

}  // namespace robot_body
