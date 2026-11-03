#include "mass_table.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {
constexpr double kTol = 1e-9;  // grid values closer than this are the same

// Index i with grid[i] <= x <= grid[i + 1], or -1 outside the grid.
int cellIndex(const std::vector<double>& grid, double x) {
  if (grid.size() < 2 || x < grid.front() - kTol || x > grid.back() + kTol) return -1;
  const auto it = std::upper_bound(grid.begin(), grid.end(), x);
  const int i = static_cast<int>(it - grid.begin()) - 1;
  return std::min(std::max(i, 0), static_cast<int>(grid.size()) - 2);
}

int indexOf(const std::vector<double>& grid, double x) {
  const auto it = std::lower_bound(grid.begin(), grid.end(), x - kTol);
  return static_cast<int>(it - grid.begin());
}
}  // namespace

MassTable MassTable::load(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open reflected mass table " + path);

  struct Row {
    double r, z, m;
  };
  std::vector<Row> rows;
  std::string line;
  for (int number = 1; std::getline(in, line); ++number) {
    if (number == 1 && line.rfind("\xEF\xBB\xBF", 0) == 0) line.erase(0, 3);  // UTF-8 BOM
    line = line.substr(0, line.find('#'));
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    // A header line (e.g. "az,z,m") starts with a letter.
    if (rows.empty() && std::isalpha(static_cast<unsigned char>(
                            line[line.find_first_not_of(" \t")]))) {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream f(line);
    Row row;
    if (!(f >> row.r >> row.z >> row.m) || !(row.m > 0.0)) {
      throw std::runtime_error(path + ":" + std::to_string(number) +
                               ": expected r,z,m with a positive mass");
    }
    rows.push_back(row);
  }

  MassTable t;
  auto addUnique = [](std::vector<double>* grid, double x) {
    if (std::none_of(grid->begin(), grid->end(),
                     [x](double g) { return std::abs(g - x) < kTol; })) {
      grid->push_back(x);
    }
  };
  for (const Row& row : rows) {
    addUnique(&t.r_, row.r);
    addUnique(&t.z_, row.z);
  }
  std::sort(t.r_.begin(), t.r_.end());
  std::sort(t.z_.begin(), t.z_.end());
  if (t.r_.size() < 2 || t.z_.size() < 2) {
    throw std::runtime_error(path + ": need at least two r and two z values");
  }

  t.m_.assign(t.r_.size() * t.z_.size(), NAN);
  for (const Row& row : rows) {
    t.m_[indexOf(t.z_, row.z) * t.r_.size() + indexOf(t.r_, row.r)] = row.m;
  }
  if (std::any_of(t.m_.begin(), t.m_.end(), [](double m) { return std::isnan(m); })) {
    throw std::runtime_error(path + ": incomplete grid, every r,z combination needs a row");
  }
  t.max_mass_ = *std::max_element(t.m_.begin(), t.m_.end());
  return t;
}

double MassTable::lookup(double r, double z) const {
  const int ir = cellIndex(r_, r);
  const int iz = cellIndex(z_, z);
  if (ir < 0 || iz < 0) return max_mass_;  // outside the grid: most conservative
  const size_t n = r_.size();
  return std::max({m_[iz * n + ir], m_[iz * n + ir + 1], m_[(iz + 1) * n + ir],
                   m_[(iz + 1) * n + ir + 1]});
}
