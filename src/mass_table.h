// Reflected mass of the robot for a hand trapped against the table, tabulated over the EE
// position: horizontal distance r from the base axis and height z above the table.
//
// The table is not monotonic, so a lookup takes the largest mass of the four grid points around
// (r, z); outside the grid it returns the largest mass of the whole table.
//
// File: CSV, one "r,z,m" row per grid point [m, m, kg]; an optional header line (e.g. "az,z,m")
// and a UTF-8 BOM are skipped, '#' starts a comment. Every combination of the r and z values that
// occur must be present.
#pragma once

#include <string>
#include <vector>

class MassTable {
 public:
  // Throws std::runtime_error on a missing file, a malformed line or an incomplete grid.
  static MassTable load(const std::string& path);

  double lookup(double r, double z) const;

  size_t rows() const { return z_.size(); }
  size_t cols() const { return r_.size(); }
  double maxMass() const { return max_mass_; }

 private:
  std::vector<double> r_, z_;  // sorted grid values
  std::vector<double> m_;      // m_[iz * r_.size() + ir]
  double max_mass_ = 0.0;
};
