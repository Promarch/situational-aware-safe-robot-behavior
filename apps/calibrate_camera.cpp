// Camera-robot extrinsic calibration from hand/end-effector point pairs. The robot does NOT
// move: guide it by hand (pilot buttons) and only its state is read.
//
// The detector (hand_detect_udp.py, run WITHOUT --extrinsic) sends the hand's middle-finger
// knuckle (MediaPipe landmark 9, sphere radius 0.10) in the camera frame. For every pair:
//   1. guide the robot somewhere in the camera's view,
//   2. hold the middle-finger knuckle at the center between the gripper fingertips (the EE
//      point), back of the hand towards the camera, and keep still,
//   3. press Enter: the EE position comes from the robot, the knuckle from the median of the next
//      second of detector packets (only those using landmark 9, i.e. radius 0.10).
// Spread 6-10 pairs over the workspace (different heights too, never all on a line), then enter
// q. T_base_cam is fitted with Umeyama/Kabsch (rotation + translation, no scale) and written as
// a 4x4 text file for hand_detect_udp.py --extrinsic. Each pair's residual is printed; remove bad
// pairs from the pairs file and refit with --from.
//
// Usage: calibrate_camera [port=5005]                          record pairs, then fit
//        calibrate_camera --from ../config/calib_pairs.csv     fit from saved pairs
// Writes config/calib_pairs.csv (base x,y,z, camera x,y,z per row) and config/T_base_cam.txt,
// the calibration in use (hand_detect_udp.py --extrinsic), replacing the previous ones.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <franka/exception.h>
#include <franka/robot.h>

#include "hand_receiver.h"

namespace {
const std::string kConfigDir = std::string(PROJECT_DIR) + "/config/";
constexpr const char* kRobotIp = "192.168.3.8";
constexpr double kKnuckleRadius = 0.10;   // detector's radius when it sends landmark 9
constexpr double kSampleSeconds = 1.0;
constexpr int kMinSamples = 15;           // ~half a second at 30 fps
constexpr double kMaxSpread = 0.01;       // max std of the knuckle while holding still [m]
constexpr int kMinPairs = 4;

struct Pair {
  Eigen::Vector3d base, cam;
};

double median(std::vector<double> v) {
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

bool fitAndWrite(const std::vector<Pair>& pairs) {
  if (static_cast<int>(pairs.size()) < kMinPairs) {
    std::cerr << "need at least " << kMinPairs << " pairs, have " << pairs.size() << "\n";
    return false;
  }
  Eigen::Matrix3Xd cam(3, pairs.size()), base(3, pairs.size());
  for (size_t i = 0; i < pairs.size(); ++i) {
    cam.col(i) = pairs[i].cam;
    base.col(i) = pairs[i].base;
  }
  // Points far from a line are needed for a unique rotation.
  const Eigen::Matrix3Xd centered = base.colwise() - base.rowwise().mean();
  const Eigen::Vector3d sv = Eigen::JacobiSVD<Eigen::Matrix3Xd>(centered).singularValues();
  if (sv(1) < 0.05 * std::sqrt(pairs.size())) {
    std::cerr << "WARNING: the points lie almost on a line (singular values " << sv.transpose()
              << "); spread them out more\n";
  }

  const Eigen::Matrix4d T = Eigen::umeyama(cam, base, false);  // base = T * cam
  std::cout << std::fixed << std::setprecision(4) << "\nResiduals (base frame):\n";
  double sum_sq = 0.0, worst = 0.0;
  for (size_t i = 0; i < pairs.size(); ++i) {
    const Eigen::Vector3d fitted = T.topLeftCorner<3, 3>() * pairs[i].cam + T.topRightCorner<3, 1>();
    const double e = (fitted - pairs[i].base).norm();
    sum_sq += e * e;
    worst = std::max(worst, e);
    std::cout << "  pair " << i + 1 << ": " << e * 100 << " cm\n";
  }
  std::cout << "RMS " << std::sqrt(sum_sq / pairs.size()) * 100 << " cm, worst " << worst * 100
            << " cm\n\nT_base_cam =\n"
            << T << "\ncamera position in the base frame: " << T.topRightCorner<3, 1>().transpose()
            << "\ncamera viewing direction (z) in the base frame: "
            << T.block<3, 1>(0, 2).transpose() << "\n";

  const std::string path = kConfigDir + "T_base_cam.txt";
  std::ofstream out(path);
  if (!out) {
    std::cerr << "cannot write " << path << "\n";
    return false;
  }
  out << std::setprecision(9);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) out << T(r, c) << (c < 3 ? " " : "\n");
  }
  std::cout << "Wrote " << path << " (for hand_detect_udp.py --extrinsic; restart the "
               "detector to use it)\n";
  return true;
}

std::vector<Pair> loadPairs(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::vector<Pair> pairs;
  std::string line;
  while (std::getline(in, line)) {
    line = line.substr(0, line.find('#'));
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream f(line);
    Pair p;
    if (f >> p.base.x() >> p.base.y() >> p.base.z() >> p.cam.x() >> p.cam.y() >> p.cam.z()) {
      pairs.push_back(p);
    }
  }
  return pairs;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 2 && std::strcmp(argv[1], "--from") == 0) {
      return fitAndWrite(loadPairs(argv[2])) ? 0 : 1;
    }
    const auto port = static_cast<uint16_t>(argc > 1 ? std::atoi(argv[1]) : 5005);

    franka::Robot robot(kRobotIp);

    // Knuckle samples in the camera frame, collected while `collecting` is set.
    std::mutex mutex;
    bool collecting = false;
    std::vector<Eigen::Vector3d> samples;
    int skipped = 0;  // frames without exactly one hand using landmark 9
    HandReceiver receiver(
        port,
        [&](const supervision::HandObservation& obs) {
          std::lock_guard<std::mutex> lock(mutex);
          if (!collecting) return;
          if (obs.n_hands == 1 && !obs.unlocated &&
              std::abs(obs.hands[0].radius - kKnuckleRadius) < 1e-4) {
            samples.push_back(obs.hands[0].center);
          } else {
            ++skipped;
          }
        },
        std::chrono::milliseconds(100), /*accept_any_frame=*/true);

    std::vector<Pair> pairs;
    const std::string pairs_path = kConfigDir + "calib_pairs.csv";
    std::ofstream pairs_file(pairs_path);
    pairs_file << "# base_x,base_y,base_z,cam_x,cam_y,cam_z  [m]\n" << std::setprecision(6);
    std::cout << "Listening on UDP port " << port << ". Run the detector WITHOUT --extrinsic.\n";

    while (true) {
      std::cout << "\nPair " << pairs.size() + 1
                << ": guide the robot, hold the middle-finger knuckle at the fingertips, keep "
                   "still and press Enter (q + Enter to finish) "
                << std::flush;
      std::string line;
      if (!std::getline(std::cin, line) || line == "q") break;

      const std::array<double, 16> T = robot.readOnce().O_T_EE;
      const Eigen::Vector3d base(T[12], T[13], T[14]);
      {
        std::lock_guard<std::mutex> lock(mutex);
        samples.clear();
        skipped = 0;
        collecting = true;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(kSampleSeconds));
      std::vector<Eigen::Vector3d> got;
      int bad = 0;
      {
        std::lock_guard<std::mutex> lock(mutex);
        collecting = false;
        got = samples;
        bad = skipped;
      }

      if (static_cast<int>(got.size()) < kMinSamples) {
        std::cout << "  rejected: only " << got.size() << " knuckle samples (" << bad
                  << " frames without exactly one hand using landmark 9). Is the detector "
                     "running and the back of the hand visible?\n";
        continue;
      }
      Eigen::Vector3d cam, spread;
      for (int k = 0; k < 3; ++k) {
        std::vector<double> c;
        for (const Eigen::Vector3d& s : got) c.push_back(s(k));
        cam(k) = median(c);
        double var = 0.0;
        for (double x : c) var += (x - cam(k)) * (x - cam(k));
        spread(k) = std::sqrt(var / c.size());
      }
      std::cout << std::fixed << std::setprecision(4) << "  EE (base) " << base.transpose()
                << " | knuckle (camera) " << cam.transpose() << " | " << got.size()
                << " samples, spread " << spread.norm() * 100 << " cm";
      if (spread.norm() > kMaxSpread) {
        std::cout << " -> rejected, hand moved; try again\n";
        continue;
      }
      std::cout << "\n";
      pairs.push_back({base, cam});
      pairs_file << base.x() << "," << base.y() << "," << base.z() << "," << cam.x() << ","
                 << cam.y() << "," << cam.z() << "\n"
                 << std::flush;
    }

    std::cout << "\n" << pairs.size() << " pairs saved to " << pairs_path << "\n";
    return fitAndWrite(pairs) ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n";
    return -1;
  }
}
