#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  SwingTrajectoryPlanner::Config config;
  config.swingHeight = 0.1;
  config.liftOffVelocity = 0.05;
  config.touchDownHeightOffset = -0.001;
  config.impactProximityFactorMidPointValue = 0.1;
  SwingTrajectoryPlanner planner(config, 2);
  const ModeSchedule running({0.0, 0.3, 0.4, 0.7, 0.8, 1.1},
                             {STANCE, LF, FLY, RF, FLY, LF, STANCE});
  planner.update(running, 0.0);
  const SplineCpg expected({0.3, 0.0, 0.05}, 0.1, {0.8, -0.001, 0.0});
  for (double t : {0.31, 0.39, 0.41, 0.5, 0.69, 0.71, 0.79}) {
    assert(std::abs(planner.getZpositionConstraint(0, t) - expected.position(t)) < 1e-12);
    assert(std::abs(planner.getZvelocityConstraint(0, t) - expected.velocity(t)) < 1e-12);
    assert(std::abs(planner.getZaccelerationConstraint(0, t) - expected.acceleration(t)) < 1e-12);
  }
  for (double t : {0.4, 0.7}) {
    const double eps = 1e-8;
    assert(std::abs(planner.getZpositionConstraint(0, t - eps) - planner.getZpositionConstraint(0, t + eps)) < 1e-6);
    assert(std::abs(planner.getZvelocityConstraint(0, t - eps) - planner.getZvelocityConstraint(0, t + eps)) < 1e-6);
    assert(std::abs(planner.getImpactProximityFactor(0, t - eps) - planner.getImpactProximityFactor(0, t + eps)) < 1e-6);
  }
  // A gait change inside a swing must not select a different apex at an internal mode boundary.
  planner.update(running, 0.0, {-1.0, 0.15, 0.2, 0.3, 0.3, 0.3, -1.0});
  const SplineCpg raised({0.3, 0.0, 0.05}, 0.2, {0.8, -0.001, 0.0});
  for (double t : {0.31, 0.39, 0.41, 0.5, 0.69, 0.71, 0.79}) {
    assert(std::abs(planner.getZpositionConstraint(0, t) - raised.position(t)) < 1e-12);
    assert(std::abs(planner.getZvelocityConstraint(0, t) - raised.velocity(t)) < 1e-12);
    assert(std::abs(planner.getZaccelerationConstraint(0, t) - raised.acceleration(t)) < 1e-12);
  }
  // Terrain endpoint heights and a gait override compose, rather than replacing one another.
  feet_array_t<scalar_array_t> liftOffHeights;
  feet_array_t<scalar_array_t> touchDownHeights;
  liftOffHeights.fill(scalar_array_t(running.modeSequence.size(), 0.1));
  touchDownHeights.fill(scalar_array_t(running.modeSequence.size(), 0.2));
  planner.update(running, liftOffHeights, touchDownHeights, scalar_array_t(running.modeSequence.size(), 0.2));
  assert(std::abs(planner.getZpositionConstraint(0, 0.55) - 0.4) < 1e-12);
  for (const auto& invalid : {scalar_array_t{0.2}, scalar_array_t(7, -0.5),
                              scalar_array_t(7, std::numeric_limits<double>::quiet_NaN())}) {
    bool rejected = false;
    try {
      planner.update(running, 0.0, invalid);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    assert(rejected);
  }
  // A single-mode walking swing retains its original lift-off, apex and touchdown.
  planner.update(ModeSchedule({0.0, 0.4}, {STANCE, LF, STANCE}), 0.0);
  assert(std::abs(planner.getZpositionConstraint(1, 0.2) - 0.1) < 1e-12);
  assert(std::abs(planner.getZpositionConstraint(1, 0.4 - 1e-9) + 0.001) < 1e-8);
  planner.update(ModeSchedule({0.0, 0.4}, {STANCE, LF, STANCE}), 0.0, {-1.0, -1.0, -1.0});
  assert(std::abs(planner.getZpositionConstraint(1, 0.2) - 0.1) < 1e-12);
  planner.update(ModeSchedule({0.0, 0.4}, {STANCE, LF, STANCE}), 0.0, {-1.0, 0.0, -1.0});
  assert(std::abs(planner.getZpositionConstraint(1, 0.2)) < 1e-12);
  // H2's 0.3 s walking swing retains the duration scaling: 0.1 * 0.3 / 0.4.
  config.swingTimeScale = 0.4;
  SwingTrajectoryPlanner scaledPlanner(config, 2);
  scaledPlanner.update(ModeSchedule({0.0, 0.3}, {STANCE, LF, STANCE}), 0.0, {-1.0, 0.1, -1.0});
  assert(std::abs(scaledPlanner.getZpositionConstraint(1, 0.15) - 0.075) < 1e-12);
  // Standing needs no event times or swing endpoint lookups.
  planner.update(ModeSchedule({}, {STANCE}), 0.25);
  assert(std::abs(planner.getZpositionConstraint(0, 1.0) - 0.25) < 1e-12);
  assert(std::abs(planner.getZpositionConstraint(1, 1.0) - 0.25) < 1e-12);
}
