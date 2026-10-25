#include <cassert>
#include <cmath>

#include <humanoid_common_mpc/gait/GaitSchedule.h>
#include <humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  ModeSequenceTemplate stance({0.0, 0.5}, {STANCE});
  ModeSequenceTemplate walk({0.0, 0.3, 0.4, 0.7, 0.8}, {LF, STANCE, RF, STANCE});
  ModeSequenceTemplate run({0.0, 0.28, 0.34, 0.62, 0.68}, {LF, FLY, RF, FLY});
  walk.swingHeight = 0.1;
  run.swingHeight = 0.2;
  auto copied = walk;
  swap(copied, run);
  assert(copied.swingHeight == 0.2 && run.swingHeight == 0.1);
  swap(copied, run);

  GaitSchedule schedule(ModeSchedule({0.0, 0.5}, {STANCE, STANCE, STANCE}), stance, 0.1);
  schedule.insertModeSequenceTemplate(walk, 1.0, 2.4);
  schedule.insertModeSequenceTemplate(run, 3.0, 5.5);
  SwingTrajectoryPlanner::Config config;
  config.swingHeight = 0.08;
  config.swingTimeScale = 0.01;
  SwingTrajectoryPlanner planner(config, 2);

  const auto checkWindow = [&](double start, double end) {
    const auto modes = schedule.getModeSchedule(start, end);
    const auto& heights = schedule.getSwingHeightSequence();
    assert(heights.size() == modes.modeSequence.size());
    planner.update(modes, 0.0, heights);
    for (size_t foot = 0; foot < 2; ++foot) {
      for (size_t i = 1; i < modes.modeSequence.size(); ++i) {
        if (modeNumber2StanceLeg(modes.modeSequence[i])[foot] ||
            !modeNumber2StanceLeg(modes.modeSequence[i - 1])[foot]) {
          continue;
        }
        size_t j = i;
        while (j < modes.modeSequence.size() && !modeNumber2StanceLeg(modes.modeSequence[j])[foot]) {
          ++j;
        }
        assert(j < modes.modeSequence.size());
        const double midpoint = 0.5 * (modes.eventTimes[i - 1] + modes.eventTimes[j - 1]);
        const double expected = heights[i] < 0.0 ? config.swingHeight : heights[i];
        assert(std::abs(planner.getZpositionConstraint(foot, midpoint) - expected) < 1e-12);
      }
    }
    return modes;
  };

  const auto initial = checkWindow(-0.5, 6.0);
  bool sawWalk = false;
  bool sawRun = false;
  for (size_t i = 1; i < initial.modeSequence.size(); ++i) {
    if (initial.modeSequence[i] == STANCE) {
      continue;
    }
    const double start = initial.eventTimes[i - 1];
    const double height = schedule.getSwingHeightSequence()[i];
    if (start >= 1.0 && start < 3.0) {
      assert(height == 0.1);
      sawWalk = true;
    } else if (start >= 3.0) {
      assert(height == 0.2);
      sawRun = true;
    }
  }
  assert(sawWalk && sawRun);
  checkWindow(2.5, 7.0);  // Rolling-horizon pruning and extension preserve alignment.

  ModeSequenceTemplate trot({0.0, 0.5, 1.0}, {LF, RF});
  trot.swingHeight = 0.15;
  schedule.insertModeSequenceTemplate(trot, 4.02, 6.5);
  checkWindow(2.5, 8.0);  // Replace future running phases and add transitional stance.
  bool sawTrot = false;
  for (double height : schedule.getSwingHeightSequence()) {
    sawTrot = sawTrot || height == 0.15;
  }
  assert(sawTrot);

  for (int step = 0; step < 100; ++step) {
    const double time = 9.0 + step * 0.7;
    const auto& gait = step % 3 == 0 ? stance : (step % 3 == 1 ? walk : run);
    schedule.insertModeSequenceTemplate(gait, time, time + 2.5);
    checkWindow(time - 0.5, time + 3.0);
  }

  schedule.updateModeSchedule(ModeSchedule({0.0, 0.4}, {STANCE, LF, STANCE}));
  const auto& heights = schedule.getSwingHeightSequence();
  assert(heights == scalar_array_t(3, -1.0));
  planner.update(schedule.getCurrentModeSchedule(), 0.0, heights);
  assert(std::abs(planner.getZpositionConstraint(1, 0.2) - 0.08) < 1e-12);
}
