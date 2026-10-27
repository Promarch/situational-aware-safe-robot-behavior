// Virtual hand for testing the slowdown without a camera.
//
// A schedule is a list of appearances, each timed from the start of one of the four segments
// of the pick sequence (segment ids 1..4 in the order they run):
//   0 home -> pre-pick (joint)   1 down (vertical)   2 up (vertical)   3 pre-pick -> home (joint)
// The hand holds still at its position while it is present. Schedules are plain CSV files
// (see config/hand_schedule.csv) so tests can be tuned without recompiling.
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "supervision.h"

namespace virtual_hand {

constexpr int kNumSegments = 4;
extern const char* const kSegmentNames[kNumSegments];

struct Event {
  int segment;          // 0..3, see above
  double t_on, t_off;   // relative to the segment start [s]
  Eigen::Vector3d position;  // base frame [m]
};

// Reads "segment,t_on,t_off,x,y,z" lines; '#' starts a comment. Throws on malformed lines.
std::vector<Event> load(const std::string& path);

// Writes a schedule in the same format.
void print(const std::vector<Event>& events, std::ostream& out);

// Random schedule near the nominal path: once while the first segment is still accelerating,
// once during the descent, plus `extra` random appearances (some too far away to intrude).
// nominal_ee(segment, t) is the nominal EE position t seconds into a segment.
std::vector<Event> random(unsigned seed, int extra,
                          const std::function<Eigen::Vector3d(int, double)>& nominal_ee,
                          const std::array<double, kNumSegments>& nominal_duration);

class Hand {
 public:
  explicit Hand(std::vector<Event> events);

  // Call once per camera frame with the latest snapshot; tracks when each segment started.
  supervision::HandObservation observe(const supervision::Snapshot& snapshot, double now);

  // Hand at wall time `now` given the segment starts seen so far (thread-safe).
  supervision::HandObservation at(double now) const;

 private:
  std::vector<Event> events_;
  std::array<std::atomic<double>, kNumSegments> segment_start_;
};

}  // namespace virtual_hand
