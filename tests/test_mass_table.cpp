// Offline test of MassTable: loading, the max of the four surrounding grid points, the grid edges
// and outside the grid.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mass_table.h"

namespace {
bool ok = true;

void expect(double got, double want, const std::string& what) {
  const bool pass = std::abs(got - want) < 1e-12;
  std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] " << what << ": " << got;
  if (!pass) std::cout << " (expected " << want << ")";
  std::cout << "\n";
  ok &= pass;
}

void expectThrow(const std::string& path, const std::string& what) {
  try {
    MassTable::load(path);
    std::cout << "  [FAIL] " << what << ": no exception\n";
    ok = false;
  } catch (const std::runtime_error& e) {
    std::cout << "  [PASS] " << what << ": " << e.what() << "\n";
  }
}
}  // namespace

int main() {
  // Grid r = {0.3, 0.5, 0.7}, z = {0.0, 0.1}, deliberately not monotonic and given out of order.
  //            r=0.3  r=0.5  r=0.7
  //   z=0.0     2.0    5.0    1.0
  //   z=0.1     3.0    1.5    4.0
  {
    std::ofstream f("mass_table_test.csv");
    f << "# r,z,m\n0.5,0.0,5.0\n0.3,0.0,2.0\n0.7,0.0,1.0\n0.3,0.1,3.0\n0.5,0.1,1.5\n"
         "0.7,0.1,4.0   # last\n";
  }
  const MassTable t = MassTable::load("mass_table_test.csv");
  std::cout << "loaded " << t.cols() << " x " << t.rows() << " grid, max " << t.maxMass() << "\n";
  expect(t.lookup(0.4, 0.05), 5.0, "inside the left cell -> max of 2, 5, 3, 1.5");
  expect(t.lookup(0.6, 0.05), 5.0, "inside the right cell -> max of 5, 1, 1.5, 4");
  expect(t.lookup(0.3, 0.0), 5.0, "on the corner (0.3, 0.0) -> its cell");
  expect(t.lookup(0.7, 0.1), 5.0, "on the far corner -> last cell");
  expect(t.lookup(0.2, 0.05), 5.0, "r below the grid -> table max");
  expect(t.lookup(0.4, 0.2), 5.0, "z above the grid -> table max");

  {
    std::ofstream f("mass_table_test2.csv");  // only the right cell differs from above
    f << "0.3,0.0,1.0\n0.5,0.0,1.0\n0.7,0.0,2.5\n0.3,0.1,1.0\n0.5,0.1,1.0\n0.7,0.1,1.2\n";
  }
  const MassTable t2 = MassTable::load("mass_table_test2.csv");
  expect(t2.lookup(0.35, 0.02), 1.0, "a low cell does not take the high neighbouring cell");
  expect(t2.lookup(0.65, 0.02), 2.5, "the cell with the high corner");

  {
    std::ofstream f("mass_table_bad.csv");
    f << "0.3,0.0,1.0\n0.5,0.0,1.0\n0.3,0.1,1.0\n";  // (0.5, 0.1) missing
  }
  expectThrow("mass_table_bad.csv", "incomplete grid");
  expectThrow("does_not_exist.csv", "missing file");

  std::cout << (ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << "\n";
  return ok ? 0 : 1;
}
