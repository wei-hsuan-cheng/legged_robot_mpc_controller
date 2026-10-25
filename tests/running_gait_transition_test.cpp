#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include <ocs2_oc/trajectory_adjustment/TrajectorySpreading.h>

int main() {
  using namespace ocs2;
  // Exercise warm-start truncation when a flight gait changes to supported walking.
  for (int trial = 0; trial < 200; ++trial) {
    const double init = 0.005 * trial;
    const double end = init + 1.0;
    ModeSchedule oldSchedule;
    oldSchedule.modeSequence.push_back(2);
    for (int i = 0; i < 20; ++i) {
      oldSchedule.eventTimes.push_back(0.3 * i + 0.25);
      oldSchedule.modeSequence.push_back(0);
      oldSchedule.eventTimes.push_back(0.3 * i + 0.3);
      oldSchedule.modeSequence.push_back(i % 2 ? 2 : 1);
    }
    ModeSchedule nextSchedule = oldSchedule;
    const auto it = std::upper_bound(nextSchedule.eventTimes.begin(), nextSchedule.eventTimes.end(), init + 0.3);
    const double splice = *it;
    const size_t index = it - nextSchedule.eventTimes.begin();
    nextSchedule.eventTimes.resize(index);
    nextSchedule.modeSequence.resize(index + 1);
    double event = splice;
    for (int i = 0; i < 12; ++i) {
      nextSchedule.eventTimes.push_back(event);
      nextSchedule.modeSequence.push_back(i % 2 ? 3 : (i % 4 ? 1 : 2));
      event += i % 2 ? 0.1 : 0.3;
    }
    scalar_array_t times;
    for (double t = init; t < end; t += 0.02) times.push_back(t);
    times.push_back(end);
    for (double t : oldSchedule.eventTimes) {
      if (t > init && t < end) {
        times.push_back(t);
        times.push_back(t + 1e-9);
      }
    }
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
    vector_array_t trajectory(times.size(), vector_t::Ones(64));
    TrajectorySpreading spreading;
    spreading.set(oldSchedule, nextSchedule, times);
    spreading.adjustTrajectory(trajectory);
    spreading.adjustTimeTrajectory(times);
    assert(trajectory.size() == times.size());
    assert(std::is_sorted(times.begin(), times.end()));
    for (const auto& value : trajectory) assert(value.size() == 64 && value.allFinite());
  }
  std::cout << "Running gait warm-start transitions passed\n";
}
