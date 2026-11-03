// Compares, along the nominal pick-and-place path, the mass the energy limit uses for a free hand
// (the model's effective mass m_eff) with the one for a hand trapped against the table (the
// tabulated reflected mass m_table). The robot does NOT move: only its model is read.
//
// m_eff is taken as the supervisor does: at the EE before the grasp and after the release, at the
// screwdriver's tip in between, along the planned motion direction. m_table is looked up at the
// EE position (az = sqrt(x^2 + y^2), height above the table), nearest data outside the grid.
//
// Usage: compare_mass      writes compare_mass.csv; plot with scripts/plot_compare_mass.py
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "mass_table.h"
#include "poses.h"
#include "s_curve.h"
#include "supervision.h"

namespace {
constexpr const char* kRobotIp = "192.168.3.8";
const Eigen::Vector3d kObjectTip(0.15, 0.0, 0.0);  // as in pick_place
constexpr int kSamples = 60;                       // per motion
constexpr double kLineStep = 2e-3;                 // vertical path integration step [m]

using Jacobian = Eigen::Matrix<double, 6, 7>;
using Vec7 = Eigen::Matrix<double, 7, 1>;

struct Motion {
  std::string name;
  bool at_tip;                      // reference point: screwdriver tip, else EE
  std::vector<scurve::Vec7> qs;     // joints along the motion
  std::vector<Eigen::Vector3d> us;  // planned motion direction of the reference point there
};
}  // namespace

int main() {
  try {
    franka::Robot robot(kRobotIp);
    const franka::Model model = robot.loadModel();
    const franka::RobotState state = robot.readOnce();
    const MassTable table =
        MassTable::load(std::string(PROJECT_DIR) + "/config/constrained_collision_mass.csv");

    supervision::Params params;
    params.object_tip = kObjectTip;
    params.table_z = poses::kTableZ;
    supervision::Shared shared;
    const supervision::Supervisor supervisor(model, state, params, &shared);

    auto jacobian = [&](const scurve::Vec7& q, bool tip) {
      return Jacobian(Eigen::Map<const Jacobian>(
          model.zeroJacobian(franka::Frame::kEndEffector, q, supervisor.flangeToReference(tip),
                             state.EE_T_K)
              .data()));
    };
    auto eePosition = [&](const scurve::Vec7& q) {
      const std::array<double, 16> T =
          model.pose(franka::Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
      return Eigen::Vector3d(T[12], T[13], T[14]);
    };

    // Joint motion: straight joint line, direction J * delta.
    auto jointMotion = [&](const std::string& name, const scurve::Vec7& a,
                           const scurve::Vec7& b, bool tip) {
      Motion m{name, tip, {}, {}};
      Vec7 delta;
      for (int i = 0; i < 7; ++i) delta[i] = b[i] - a[i];
      for (int k = 0; k <= kSamples; ++k) {
        scurve::Vec7 q;
        for (int i = 0; i < 7; ++i) q[i] = a[i] + delta[i] * k / kSamples;
        m.qs.push_back(q);
        m.us.push_back((jacobian(q, tip).topRows<3>() * delta).normalized());
      }
      return m;
    };
    // Vertical motion of the EE by dz from q0, followed with the model (resolved rate).
    auto verticalMotion = [&](const std::string& name, const scurve::Vec7& q0, double dz,
                              bool tip, scurve::Vec7* q_end) {
      Motion m{name, tip, {}, {}};
      scurve::Vec7 q = q0;
      const int steps = static_cast<int>(std::ceil(std::abs(dz) / kLineStep));
      const int every = std::max(1, steps / kSamples);
      for (int k = 0; k <= steps; ++k) {
        if (k % every == 0 || k == steps) {
          m.qs.push_back(q);
          m.us.push_back(Eigen::Vector3d(0, 0, dz > 0 ? 1.0 : -1.0));
        }
        if (k == steps) break;
        const Jacobian J = jacobian(q, false);
        Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
        twist[2] = dz / steps;
        const Vec7 dq = J.transpose() * (J * J.transpose() +
                                         1e-6 * Eigen::Matrix<double, 6, 6>::Identity())
                                            .ldlt()
                                            .solve(twist);
        for (int i = 0; i < 7; ++i) q[i] += dq[i];
      }
      *q_end = q;
      return m;
    };

    const double z_pre_pick = eePosition(poses::kPrePickJoints).z();
    const double z_grasp = poses::kTableZ + poses::kGraspHeight;
    scurve::Vec7 q_grasp, q_back, q_placed, q_back2;
    std::vector<Motion> motions;
    motions.push_back(jointMotion("start->pre-pick", poses::kStart, poses::kPrePickJoints, false));
    motions.push_back(verticalMotion("down to pick", poses::kPrePickJoints, z_grasp - z_pre_pick,
                                     false, &q_grasp));
    motions.push_back(
        verticalMotion("up from pick", q_grasp, z_pre_pick - z_grasp, true, &q_back));
    motions.push_back(
        jointMotion("pre-pick->pre-place", poses::kPrePickJoints, poses::kPrePlaceJoints, true));
    motions.push_back(verticalMotion("down to place", poses::kPrePlaceJoints,
                                     -poses::kPlaceDescent, true, &q_placed));
    motions.push_back(
        verticalMotion("up from place", q_placed, poses::kPlaceDescent, false, &q_back2));
    motions.push_back(jointMotion("pre-place->start", poses::kPrePlaceJoints, poses::kStart, false));

    std::ofstream csv("compare_mass.csv");
    csv << std::setprecision(6) << "motion,progress,x,y,z,az,m_eff,m_table\n";
    std::cout << std::fixed << std::setprecision(2)
              << "motion                 ref   EE az [m]     EE z [m]      m_eff [kg]     "
                 "m_table [kg]   m_table/m_eff\n";
    for (size_t mi = 0; mi < motions.size(); ++mi) {
      const Motion& m = motions[mi];
      double az_min = 1e9, az_max = 0, z_min = 1e9, z_max = -1e9, me_min = 1e9, me_max = 0,
             mt_min = 1e9, mt_max = 0, ratio_min = 1e9, ratio_max = 0;
      for (size_t k = 0; k < m.qs.size(); ++k) {
        const Eigen::Vector3d p = eePosition(m.qs[k]);
        const double az = std::hypot(p.x(), p.y());
        const double z = p.z() - poses::kTableZ;
        const double m_eff = supervisor.effectiveMass(m.qs[k], m.us[k], m.at_tip);
        const double m_table = table.lookup(az, z);
        csv << mi << "," << static_cast<double>(k) / (m.qs.size() - 1) << "," << p.x() << ","
            << p.y() << "," << p.z() << "," << az << "," << m_eff << "," << m_table << "\n";
        az_min = std::min(az_min, az), az_max = std::max(az_max, az);
        z_min = std::min(z_min, z), z_max = std::max(z_max, z);
        me_min = std::min(me_min, m_eff), me_max = std::max(me_max, m_eff);
        mt_min = std::min(mt_min, m_table), mt_max = std::max(mt_max, m_table);
        ratio_min = std::min(ratio_min, m_table / m_eff);
        ratio_max = std::max(ratio_max, m_table / m_eff);
      }
      std::cout << std::left << std::setw(22) << m.name << " " << (m.at_tip ? "tip" : "EE ")
                << std::right << "   " << az_min << ".." << az_max << "    " << z_min << ".."
                << z_max << "     " << me_min << ".." << me_max << "     " << mt_min << ".."
                << mt_max << "     " << ratio_min << ".." << ratio_max << "\n";
    }
    std::cout << "\nTable grid: az " << table.cols() << " values, z " << table.rows()
              << " values; outside it the nearest data is used.\nWrote compare_mass.csv (plot: "
                 "python3 ../scripts/plot_compare_mass.py compare_mass.csv)\n";
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
