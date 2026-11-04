// Offline test of robot_body: segment geometry, capsule placement and the shift of a frame
// Jacobian to a point. No robot needed (the libfranka Jacobian itself is checked on the robot by
// show_capsules).
#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Geometry>

#include "robot_body.h"

namespace {
bool ok = true;

void expect(bool pass, const std::string& what) {
  std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] " << what << "\n";
  ok &= pass;
}
void expectNear(double got, double want, const std::string& what, double tol = 1e-9) {
  const bool pass = std::abs(got - want) < tol;
  std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] " << what << ": " << got;
  if (!pass) std::cout << " (expected " << want << ")";
  std::cout << "\n";
  ok &= pass;
}
void expectNear(const Eigen::Vector3d& got, const Eigen::Vector3d& want, const std::string& what,
                double tol = 1e-9) {
  const bool pass = (got - want).norm() < tol;
  std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] " << what << ": " << got.transpose();
  if (!pass) std::cout << " (expected " << want.transpose() << ")";
  std::cout << "\n";
  ok &= pass;
}

}  // namespace

int main() {
  using robot_body::Capsule;
  using V = Eigen::Vector3d;

  std::cout << "closestOnSegment\n";
  expectNear(robot_body::closestOnSegment(V(0, 0, 0), V(1, 0, 0), V(0.3, 1, 0)), 0.3, "inside");
  expectNear(robot_body::closestOnSegment(V(0, 0, 0), V(1, 0, 0), V(-1, 1, 0)), 0.0, "before a");
  expectNear(robot_body::closestOnSegment(V(0, 0, 0), V(1, 0, 0), V(2, -1, 0)), 1.0, "after b");
  expectNear(robot_body::closestOnSegment(V(1, 1, 1), V(1, 1, 1), V(0, 0, 0)), 0.0, "degenerate");

  std::cout << "segmentChord\n";
  double t0 = -1, t1 = -1;
  expect(robot_body::segmentChord(V(0, 0, 0), V(1, 0, 0), V(0.5, 0.3, 0), 0.5, &t0, &t1),
         "crossing");
  expectNear(t0, 0.1, "crossing t0");
  expectNear(t1, 0.9, "crossing t1");
  expect(robot_body::segmentChord(V(0, 0, 0), V(1, 0, 0), V(0.5, 0.3, 0), 0.3, &t0, &t1),
         "tangent");
  expectNear(t0, 0.5, "tangent t0", 1e-6);
  expectNear(t1, 0.5, "tangent t1", 1e-6);
  expect(!robot_body::segmentChord(V(0, 0, 0), V(1, 0, 0), V(0.5, 0.3, 0), 0.2, &t0, &t1),
         "outside");
  expect(robot_body::segmentChord(V(0, 0, 0), V(1, 0, 0), V(0.5, 0, 0), 2.0, &t0, &t1),
         "segment inside the sphere");
  expectNear(t0, 0.0, "clamped t0");
  expectNear(t1, 1.0, "clamped t1");
  expect(!robot_body::segmentChord(V(0, 0, 0), V(1, 0, 0), V(2, 0, 0), 0.5, &t0, &t1),
         "sphere beyond b");

  std::cout << "surfacePoint, nearestCapsule\n";
  const Capsule cx{V(0, 0, 0), V(1, 0, 0), 0.1, robot_body::kLink4, 3};
  expectNear(robot_body::surfacePoint(cx, 0.5, V(0.5, 1, 0)), V(0.5, 0.1, 0), "facing the point");
  expectNear((robot_body::surfacePoint(cx, 0.5, V(0.7, 0, 0)) - V(0.5, 0, 0)).norm(), 0.1,
             "point on the axis: any side at the radius");
  const Capsule cz{V(2, 0, 0), V(2, 0, 1), 0.05, robot_body::kHand, 7};
  const robot_body::Nearest n = robot_body::nearestCapsule({cx, cz}, V(1.8, 0, 0.5));
  expect(n.index == 1, "nearest is the vertical capsule");
  expectNear(n.distance, 0.15, "distance to its surface");
  expectNear(n.point, V(1.95, 0, 0.5), "nearest surface point");

  std::cout << "placeCapsules\n";
  robot_body::Frames frames;
  for (Eigen::Isometry3d& T : frames) T = Eigen::Isometry3d::Identity();
  frames[3] = Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
  frames[robot_body::kEndEffector] = Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1));
  robot_body::BodyConfig config;
  config.object_tip = V(0.15, 0, 0);
  config.object_radius = 0.015;
  const std::vector<Capsule> free_hand = robot_body::placeCapsules(frames, false, config);
  const std::vector<Capsule> holding = robot_body::placeCapsules(frames, true, config);
  expect(free_hand.size() == robot_body::kObject, "links 2-7 and the hand, no object, no point");
  expect(holding.size() == robot_body::kObject + 1 && holding.back().id == robot_body::kObject,
         "object capsule while holding");
  bool ids_in_order = true;
  for (size_t i = 0; i < holding.size(); ++i) ids_in_order &= holding[i].id == i;
  expect(ids_in_order, "capsule ids in order");
  // link4: (0,0,-0.06)..(0,0,0.06) in frame 4, rotated +90 deg about x: z -> -y.
  expectNear(free_hand[robot_body::kLink4].a, V(1, 2.06, 3), "link4 a in the base frame");
  expectNear(free_hand[robot_body::kLink4].b, V(1, 1.94, 3), "link4 b in the base frame");
  expectNear(holding.back().a, V(0, 0, 1), "object starts at the EE");
  expectNear(holding.back().b, V(0.15, 0, 1), "object ends at the tip");
  // Hand capsules on the flange (identity here): along the hand's y axis (flange turned -45 deg).
  const double c45 = std::sqrt(0.5);
  expectNear(free_hand[robot_body::kHand].a, V(-0.05 * c45, -0.05 * c45, 0.04), "hand a");
  expectNear(free_hand[robot_body::kHand].b, V(0.05 * c45, 0.05 * c45, 0.04), "hand b");
  expectNear(free_hand[robot_body::kFingers].a.z(), 0.10, "fingers 0.10 above the flange");

  robot_body::BodyConfig points_only = config;
  points_only.check_links = false;
  const std::vector<Capsule> point = robot_body::placeCapsules(frames, false, points_only);
  const std::vector<Capsule> tip = robot_body::placeCapsules(frames, true, points_only);
  expect(point.size() == 1 && point[0].id == robot_body::kReference && point[0].radius == 0.0,
         "check_links off: only the reference point");
  expectNear(point[0].a, V(0, 0, 1), "reference point at the EE");
  expect(tip.size() == 1, "check_links off while holding: still one point");
  expectNear(tip[0].a, V(0.15, 0, 1), "reference point at the object tip while holding");

  robot_body::BodyConfig no_hand = config;
  no_hand.franka_hand = false;
  const std::vector<Capsule> other_tool = robot_body::placeCapsules(frames, false, no_hand);
  bool hand_missing = other_tool.size() == robot_body::kHand + 1 &&
                      other_tool.back().id == robot_body::kReference;
  for (const Capsule& c : other_tool) hand_missing &= c.id != robot_body::kHand && c.id != robot_body::kFingers;
  expect(hand_missing, "franka_hand off: no hand capsules, the reference point instead");

  robot_body::BodyConfig safety = config;
  safety.safety_distance = 0.01;
  const std::vector<Capsule> inflated = robot_body::placeCapsules(frames, true, safety);
  expectNear(inflated[robot_body::kLink4].radius, 0.07, "safety distance on link4");
  expectNear(inflated.back().radius, 0.025, "safety distance on the object");

  std::cout << "shiftToPoint (libfranka frame Jacobian -> point)\n";
  {
    // A frame Jacobian whose joint 1 only turns about z: a point 1 m along x moves along y.
    robot_body::Jacobian J = robot_body::Jacobian::Zero();
    J(5, 0) = 1.0;
    const robot_body::LinearJacobian Jp = robot_body::shiftToPoint(J, V(1, 0, 0));
    expectNear(V(Jp.col(0)), V(0, 1, 0), "rotation about z moves a point on x along y");
    // Any Jacobian: v_p = v_o + w x r for any joint velocity.
    const robot_body::Jacobian A = robot_body::Jacobian::Random();
    const Eigen::Matrix<double, 7, 1> dq = Eigen::Matrix<double, 7, 1>::Random();
    const V r(0.1, -0.2, 0.3);
    const V v_o = A.topRows<3>() * dq, w = A.bottomRows<3>() * dq;
    expectNear(V(robot_body::shiftToPoint(A, r) * dq), V(v_o + w.cross(r)), "v_p = v_o + w x r");
    expectNear(V(robot_body::shiftToPoint(A, V::Zero()) * dq), v_o, "no offset: the frame origin");
  }

  std::cout << (ok ? "ALL PASSED\n" : "FAILURES\n");
  return ok ? 0 : 1;
}
