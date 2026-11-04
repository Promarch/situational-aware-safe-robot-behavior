#include "robot_body.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_body {

namespace {
// Capsule from franka_description's form: center, unit axis, length (between the two sphere
// centers), radius.
CapsuleSpec spec(uint8_t id, int frame, const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
                 double length, double radius) {
  return {id, frame, center - 0.5 * length * axis, center + 0.5 * length * axis, radius};
}

// Any unit vector perpendicular to the unit vector d.
Eigen::Vector3d perpendicular(const Eigen::Vector3d& d) {
  const Eigen::Vector3d helper =
      std::abs(d.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  return d.cross(helper).normalized();
}

Eigen::Vector3d axisDirection(const Capsule& c) {
  const Eigen::Vector3d d = c.b - c.a;
  const double n = d.norm();
  return n > 1e-12 ? Eigen::Vector3d(d / n) : Eigen::Vector3d::UnitZ();
}
}  // namespace

franka::Frame frankaFrame(int index) {
  if (index == kFlange) return franka::Frame::kFlange;
  if (index == kEndEffector) return franka::Frame::kEndEffector;
  return static_cast<franka::Frame>(static_cast<int>(franka::Frame::kJoint1) + index);
}

const char* capsuleName(uint8_t id) {
  static const char* const kNames[kNumCapsules] = {
      "link2", "link3", "link4",   "link5",  "link5b",   "link6",
      "link7", "link7b", "hand", "fingers", "object", "reference"};
  return id < kNumCapsules ? kNames[id] : "none";
}

const std::array<CapsuleSpec, kObject>& fixedCapsules() {
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  const double c45 = std::sqrt(0.5);
  // link7's small side capsule: along x of link7_sc, which is link7 turned +45 deg about z.
  const Eigen::Vector3d x_link7_sc(c45, c45, 0.0);
  // Franka Hand: the hand frame is the flange turned -45 deg about z; its capsules lie along the
  // hand's y axis (the finger direction), which is (c45, c45, 0) in the flange frame.
  const Eigen::Vector3d y_hand(c45, c45, 0.0);
  static const std::array<CapsuleSpec, kObject> kSpecs = {{
      spec(kLink2, 1, {0, 0, 0}, z, 0.12, 0.06),
      spec(kLink3, 2, {0, 0, -0.145}, z, 0.15, 0.06),
      spec(kLink4, 3, {0, 0, 0}, z, 0.12, 0.06),
      spec(kLink5, 4, {0, 0, -0.26}, z, 0.10, 0.06),
      spec(kLink5b, 4, {0, 0.08, -0.13}, z, 0.14, 0.025),
      spec(kLink6, 5, {0, 0, -0.03}, z, 0.08, 0.05),
      spec(kLink7, 6, {0, 0, 0.01}, z, 0.14, 0.04),
      spec(kLink7b, 6, 0.06 * x_link7_sc + Eigen::Vector3d(0, 0, 0.082), x_link7_sc, 0.01, 0.03),
      spec(kHand, kFlange, {0, 0, 0.04}, y_hand, 0.10, 0.04),
      spec(kFingers, kFlange, {0, 0, 0.10}, y_hand, 0.10, 0.02),
  }};
  return kSpecs;
}

std::vector<Capsule> placeCapsules(const Frames& frames, bool holding, const BodyConfig& config) {
  std::vector<Capsule> out;
  out.reserve(kNumCapsules);
  const Eigen::Isometry3d& T_ee = frames[kEndEffector];
  if (config.check_links) {
    for (const CapsuleSpec& s : fixedCapsules()) {
      if (!config.franka_hand && (s.id == kHand || s.id == kFingers)) continue;
      const Eigen::Isometry3d& T = frames[s.frame];
      out.push_back({T * s.a, T * s.b, s.radius + config.safety_distance, s.id, s.frame});
    }
    if (holding) {
      out.push_back({T_ee.translation(), T_ee * config.object_tip,
                     config.object_radius + config.safety_distance, kObject, kEndEffector});
    }
  }
  if (!config.check_links || !config.franka_hand) {
    const Eigen::Vector3d p = T_ee * (holding ? config.object_tip : Eigen::Vector3d::Zero());
    out.push_back({p, p, 0.0, kReference, kEndEffector});
  }
  return out;
}

double closestOnSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                        const Eigen::Vector3d& p) {
  const Eigen::Vector3d d = b - a;
  const double dd = d.squaredNorm();
  if (dd < 1e-24) return 0.0;
  return std::clamp((p - a).dot(d) / dd, 0.0, 1.0);
}

bool segmentChord(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                  const Eigen::Vector3d& center, double R, double* t0, double* t1) {
  const Eigen::Vector3d d = b - a, f = a - center;
  const double A = d.squaredNorm(), B = 2.0 * f.dot(d), C = f.squaredNorm() - R * R;
  if (A < 1e-24) {  // degenerate segment: a point
    *t0 = *t1 = 0.0;
    return C <= 0.0;
  }
  const double disc = B * B - 4.0 * A * C;
  if (disc < 0.0) return false;
  const double root = std::sqrt(disc);
  const double lo = (-B - root) / (2.0 * A), hi = (-B + root) / (2.0 * A);
  if (lo > 1.0 || hi < 0.0) return false;
  *t0 = std::max(lo, 0.0);
  *t1 = std::min(hi, 1.0);
  return true;
}

Eigen::Vector3d surfacePoint(const Capsule& c, double t, const Eigen::Vector3d& toward) {
  const Eigen::Vector3d axis_point = pointOn(c, t);
  const Eigen::Vector3d d = axisDirection(c);
  Eigen::Vector3d n = toward - axis_point;
  n -= n.dot(d) * d;  // perpendicular to the axis: the side facing `toward`
  const double len = n.norm();
  n = len > 1e-9 ? Eigen::Vector3d(n / len) : perpendicular(d);
  return axis_point + c.radius * n;
}

Nearest nearestCapsule(const std::vector<Capsule>& capsules, const Eigen::Vector3d& p) {
  Nearest best;
  best.distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < capsules.size(); ++i) {
    const Capsule& c = capsules[i];
    const double t = closestOnSegment(c.a, c.b, p);
    const double distance = (p - pointOn(c, t)).norm() - c.radius;
    if (distance < best.distance) {
      best.index = static_cast<int>(i);
      best.distance = distance;
      best.point = surfacePoint(c, t, p);
    }
  }
  return best;
}

LinearJacobian shiftToPoint(const Jacobian& frame_jacobian, const Eigen::Vector3d& r) {
  Eigen::Matrix3d skew;
  skew << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
  return frame_jacobian.topRows<3>() - skew * frame_jacobian.bottomRows<3>();
}

RobotBody::RobotBody(const franka::Model& model, const franka::RobotState& state,
                     const BodyConfig& config)
    : model_(model), F_T_EE_(state.F_T_EE), EE_T_K_(state.EE_T_K), config_(config) {}

Frames RobotBody::frames(const scurve::Vec7& q) const {
  Frames out;
  for (int i = 0; i < kNumFrames; ++i) {
    out[i] = Eigen::Isometry3d(
        Eigen::Matrix4d::Map(model_.pose(frankaFrame(i), q, F_T_EE_, EE_T_K_).data()));
  }
  return out;
}

Jacobian RobotBody::frameJacobian(const scurve::Vec7& q, int frame) const {
  return Eigen::Map<const Jacobian>(
      model_.zeroJacobian(frankaFrame(frame), q, F_T_EE_, EE_T_K_).data());
}

}  // namespace robot_body
