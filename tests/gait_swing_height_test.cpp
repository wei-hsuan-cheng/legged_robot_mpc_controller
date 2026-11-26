#include <cassert>
#include <cmath>

#include <legged_robot_mpc_controller/mpc/common/gait/GaitSchedule.h>
#include <legged_robot_mpc_controller/mpc/common/swing_foot_planner/SwingTrajectoryPlanner.h>

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

  // Ground per contact: contact 1 already stands on a 40 mm step, contact 0 (trailing) on the floor. Contact 0's
  // swing lifts off the floor and lands on the step; contact 1's next swing stays on the step; both stand there.
  {
    size_t swing0 = STANCE;
    size_t swing1 = STANCE;
    for (size_t mode = 0; mode < 4; ++mode) {
      const auto flags = modeNumber2StanceLeg(mode);
      if (!flags[0] && flags[1]) swing0 = mode;
      if (flags[0] && !flags[1]) swing1 = mode;
    }
    const ModeSchedule steps({0.2, 0.45, 0.6, 0.85}, {STANCE, swing0, STANCE, swing1, STANCE});
    feet_array_t<scalar_array_t> liftOff, touchDown;
    contactHeightSequences(steps, {0.0, 0.04}, 0.0, liftOff, touchDown);
    SwingTrajectoryPlanner::Config flat;
    flat.swingHeight = 0.08;
    flat.swingTimeScale = 0.01;
    SwingTrajectoryPlanner stepping(flat, 2);
    stepping.update(steps, liftOff, touchDown);
    const auto z = [&](size_t foot, double time) { return stepping.getZpositionConstraint(foot, time); };
    assert(std::abs(z(0, 0.1) - 0.0) < 1e-12);     // standing on the floor
    assert(std::abs(z(0, 0.2) - 0.0) < 1e-9);      // lift-off from the floor
    assert(std::abs(z(0, 0.45) - 0.04) < 1e-9);    // touch-down on the step
    assert(std::abs(z(0, 0.5) - 0.04) < 1e-12);    // standing on the step
    assert(std::abs(z(1, 0.3) - 0.04) < 1e-12);    // the leading contact stands on the step
    assert(std::abs(z(1, 0.6) - 0.04) < 1e-9);     // and steps on it
    assert(std::abs(z(1, 0.85) - 0.04) < 1e-9);
    assert(std::abs(z(1, 0.725) - (0.04 + 0.08)) < 1e-9);  // apex above the step
    assert(std::abs(z(0, 1.0) - 0.04) < 1e-12);

    // A jump (no contact stands during the flight) lands each contact on its own ground.
    const ModeSchedule jump({0.4, 0.55}, {STANCE, 0, STANCE});
    contactHeightSequences(jump, {0.0, 0.04}, 0.0, liftOff, touchDown);
    stepping.update(jump, liftOff, touchDown);
    assert(std::abs(z(0, 0.55) - 0.0) < 1e-9 && std::abs(z(1, 0.55) - 0.04) < 1e-9);
  }
}
