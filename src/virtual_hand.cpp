#include "virtual_hand.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <random>
#include <sstream>
#include <stdexcept>

namespace virtual_hand {

const char* const kSegmentNames[kNumSegments] = {"start->pre-pick", "down", "up",
                                                 "pre-pick->start"};

std::vector<Event> load(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open hand schedule " + path);
  }
  std::vector<Event> events;
  std::string line;
  for (int number = 1; std::getline(in, line); ++number) {
    line = line.substr(0, line.find('#'));
    if (line.find_first_not_of(" \t\r") == std::string::npos) {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream fields(line);
    Event e;
    if (!(fields >> e.segment >> e.t_on >> e.t_off >> e.position.x() >> e.position.y() >>
          e.position.z()) ||
        e.segment < 0 || e.segment >= kNumSegments || e.t_off <= e.t_on) {
      throw std::runtime_error(path + ":" + std::to_string(number) + ": expected " +
                               "segment(0-3),t_on,t_off(>t_on),x,y,z");
    }
    events.push_back(e);
  }
  return events;
}

void print(const std::vector<Event>& events, std::ostream& out) {
  out << "# segment,t_on,t_off,x,y,z   (segment: 0 start->pre-pick, 1 down, 2 up, "
         "3 pre-pick->start;\n#  times from the segment start [s], position in the base frame [m])\n"
      << std::fixed << std::setprecision(3);
  for (const Event& e : events) {
    out << e.segment << "," << e.t_on << "," << e.t_off << "," << e.position.x() << ","
        << e.position.y() << "," << e.position.z() << "   # " << kSegmentNames[e.segment] << "\n";
  }
}

std::vector<Event> random(unsigned seed,
                          const std::function<Eigen::Vector3d(int, double)>& nominal_ee,
                          const std::array<double, kNumSegments>& nominal_duration) {
  std::mt19937 rng(seed);
  auto uniform = [&](double a, double b) { return std::uniform_real_distribution<>(a, b)(rng); };
  std::vector<Event> events;
  for (int seg = 0; seg < kNumSegments; ++seg) {
    // The first segment's hand appears while the robot is still accelerating; the others at a
    // random time in the first 60 % of their segment.
    const double t_on = seg == 0 ? uniform(0.05, 0.2) : uniform(0.0, 0.6 * nominal_duration[seg]);
    // On the path, where the nominal motion will be `lead` seconds after the hand appears.
    const double lead = uniform(0.2, 0.45);
    const Eigen::Vector3d p = nominal_ee(seg, std::min(t_on + lead, nominal_duration[seg]));
    events.push_back({seg, t_on, t_on + kHandDuration, p});
  }
  return events;
}

Hand::Hand(std::vector<Event> events) : events_(std::move(events)) {
  for (auto& s : segment_start_) s = -1.0;
}

supervision::HandObservation Hand::observe(const supervision::Snapshot& snapshot, double now) {
  const int seg = static_cast<int>(snapshot.segment_id) - 1;
  if (seg >= 0 && seg < kNumSegments && segment_start_[seg].load() < 0.0) {
    segment_start_[seg] = now - snapshot.t;
  }
  return at(now);
}

supervision::HandObservation Hand::at(double now) const {
  supervision::HandObservation obs;
  for (const Event& e : events_) {
    const double start = segment_start_[e.segment].load();
    if (start >= 0.0 && now >= start + e.t_on && now < start + e.t_off) {
      obs.present = true;
      obs.position = e.position;
    }
  }
  return obs;
}

}  // namespace virtual_hand
