/******************************************************************************
Copyright (c) 2026. All rights reserved.

Fixed-sequence stair climbing plan, see StairClimbingPlan.h.
******************************************************************************/

#include "humanoid_common_mpc/reference_manager/StairClimbingPlan.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace ocs2::humanoid {

namespace {

constexpr size_t LEFT_FOOT = 0;   // contact_flag_t convention {LF, RF}
constexpr size_t RIGHT_FOOT = 1;
constexpr scalar_t EPS = 1e-6;

/// Mode where the given foot is the SWING foot (the other foot is in contact).
size_t swingModeForFoot(size_t foot) {
  // ModeNumber::LF (2) == left foot in contact -> right foot swings.
  // ModeNumber::RF (1) == right foot in contact -> left foot swings.
  return (foot == LEFT_FOOT) ? ModeNumber::RF : ModeNumber::LF;
}

}  // namespace

StairClimbingPlan::StairClimbingPlan(const StairClimbingConfig& config,
                                     scalar_t startTime,
                                     const vector6_t& initialBasePose,
                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                     const ReferenceConfig& referenceConfig)
    : config_(config), startTime_(startTime) {
  const size_t numSteps = config.stepHeights.size();
  if (config.stepDepths.size() != numSteps) {
    throw std::invalid_argument("[StairClimbingPlan] stepHeights and stepDepths must have the same size.");
  }
  if (numSteps == 0) {
    throw std::invalid_argument("[StairClimbingPlan] the staircase has no steps.");
  }

  const scalar_t cosYaw = std::cos(config.stairsYaw);
  const scalar_t sinYaw = std::sin(config.stairsYaw);
  const auto toWorld = [&](scalar_t xLocal, scalar_t yLocal, scalar_t zLocal) -> vector3_t {
    return vector3_t(config.stairsBasePosition(0) + cosYaw * xLocal - sinYaw * yLocal,
                     config.stairsBasePosition(1) + sinYaw * xLocal + cosYaw * yLocal,
                     config.stairsBasePosition(2) + zLocal);
  };

  // Initial base position expressed in the staircase frame.
  const scalar_t dx = initialBasePose(0) - config.stairsBasePosition(0);
  const scalar_t dy = initialBasePose(1) - config.stairsBasePosition(1);
  const scalar_t baseStartX = cosYaw * dx + sinYaw * dy;

  const feet_array_t<scalar_t> footY{config.lateralOffset, -config.lateralOffset};

  // ---------------------------------------------------------------------------
  // 1) Generate the swing sequence in the staircase frame: (foot, x, z) targets.
  // ---------------------------------------------------------------------------
  struct LocalSwing {
    size_t foot;
    scalar_t x;
    scalar_t z;
  };
  std::vector<LocalSwing> swings;

  const scalar_t approachEnd = config.startOffset - config.approachStandoff;
  feet_array_t<scalar_t> footX{baseStartX, baseStartX};

  size_t foot = config.leftFootFirst ? LEFT_FOOT : RIGHT_FOOT;

  // Flat-ground approach: alternate feet, each foothold placed one stride ahead
  // of the other foot, capped at the stand-off line before the first riser.
  for (size_t guard = 0; guard < 100; ++guard) {
    const size_t other = 1 - foot;
    if (footX[LEFT_FOOT] > approachEnd - EPS && footX[RIGHT_FOOT] > approachEnd - EPS) {
      break;
    }
    const scalar_t targetX = std::min(footX[other] + config.approachStride, approachEnd);
    if (targetX > footX[foot] + EPS) {
      swings.push_back({foot, targetX, 0.0});
      footX[foot] = targetX;
    }
    foot = other;
  }

  const size_t leadFoot = config.leftFootFirst ? LEFT_FOOT : RIGHT_FOOT;
  scalar_t treadStartX = config.startOffset;
  scalar_t treadTopZ = 0.0;
  if (config.bothFeetPerTread) {
    // Climb: step-to gait, both feet per tread, same lead foot on every tread.
    for (size_t i = 0; i < numSteps; ++i) {
      const scalar_t footholdX = treadStartX + 0.5 * config.stepDepths[i] + config.treadInset;
      treadTopZ += config.stepHeights[i];
      swings.push_back({leadFoot, footholdX, treadTopZ});
      swings.push_back({1 - leadFoot, footholdX, treadTopZ});
      treadStartX += config.stepDepths[i];
    }
  } else {
    // Climb: one-tread-one-leg gait, alternating feet over consecutive treads
    // (each swing spans two treads), then the trailing foot joins on the top.
    size_t climbFoot = leadFoot;
    scalar_t footholdX = 0.0;
    for (size_t i = 0; i < numSteps; ++i) {
      footholdX = treadStartX + 0.5 * config.stepDepths[i] + config.treadInset;
      treadTopZ += config.stepHeights[i];
      swings.push_back({climbFoot, footholdX, treadTopZ});
      climbFoot = 1 - climbFoot;
      treadStartX += config.stepDepths[i];
    }
    swings.push_back({climbFoot, footholdX, treadTopZ});  // trailing foot joins on the top tread
  }

  // ---------------------------------------------------------------------------
  // 2) Time the sequence and build the mode sequence template + footstep lists.
  // ---------------------------------------------------------------------------
  std::vector<scalar_t> switchingTimes{0.0, config.initialStanceDuration};
  std::vector<size_t> modeSequence{ModeNumber::STANCE};

  feet_array_t<vector3_t> footPosition{toWorld(baseStartX, footY[LEFT_FOOT], 0.0),
                                       toWorld(baseStartX, footY[RIGHT_FOOT], 0.0)};

  scalar_t t = startTime_ + config.initialStanceDuration;
  for (size_t s = 0; s < swings.size(); ++s) {
    const auto& swing = swings[s];
    const vector3_t target = toWorld(swing.x, footY[swing.foot], swing.z);

    PlannedFootstep footstep;
    footstep.liftOffTime = t;
    footstep.touchDownTime = t + config.swingDuration;
    footstep.liftOffPosition = footPosition[swing.foot];
    footstep.touchDownPosition = target;
    footsteps_[swing.foot].push_back(footstep);
    footPosition[swing.foot] = target;

    modeSequence.push_back(swingModeForFoot(swing.foot));
    switchingTimes.push_back(switchingTimes.back() + config.swingDuration);

    const bool lastSwing = (s + 1 == swings.size());
    const scalar_t stance = lastSwing ? config.finalStanceDuration : config.stanceDuration;
    modeSequence.push_back(ModeNumber::STANCE);
    switchingTimes.push_back(switchingTimes.back() + stance);

    t = footstep.touchDownTime + stance;
  }

  modeSequenceTemplate_ = ModeSequenceTemplate(switchingTimes, modeSequence);
  finalTime_ = startTime_ + switchingTimes.back();

  // ---------------------------------------------------------------------------
  // 3) Base (pelvis) reference: knots at plan start and every touch-down.
  //    Zero pitch and roll; yaw fixed to the staircase heading.
  // ---------------------------------------------------------------------------
  struct BaseKnot {
    scalar_t time;
    vector3_t position;
  };
  std::vector<BaseKnot> knots;

  feet_array_t<vector3_t> feet{toWorld(baseStartX, footY[LEFT_FOOT], 0.0), toWorld(baseStartX, footY[RIGHT_FOOT], 0.0)};
  const auto baseAboveFeet = [&]() -> vector3_t {
    vector3_t mid = 0.5 * (feet[LEFT_FOOT] + feet[RIGHT_FOOT]);
    mid(2) = 0.5 * (feet[LEFT_FOOT](2) + feet[RIGHT_FOOT](2)) + config.baseHeightAboveSupport;
    return mid;
  };

  knots.push_back({startTime_, baseAboveFeet()});
  {
    feet_array_t<size_t> next{0, 0};
    for (const auto& swing : swings) {
      const auto& footstep = footsteps_[swing.foot][next[swing.foot]++];
      feet[swing.foot] = footstep.touchDownPosition;
      knots.push_back({footstep.touchDownTime, baseAboveFeet()});
    }
  }
  knots.push_back({finalTime_, knots.back().position});

  const size_t stateDim = mpcRobotModel.getStateDim();
  const size_t inputDim = mpcRobotModel.getInputDim();
  const size_t velocityIndex = mpcRobotModel.getBaseComVelocityStartindex();

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  timeTrajectory.reserve(knots.size());
  stateTrajectory.reserve(knots.size());
  inputTrajectory.reserve(knots.size());

  for (size_t k = 0; k < knots.size(); ++k) {
    vector_t state = vector_t::Zero(stateDim);

    vector6_t basePose;
    basePose << knots[k].position, config.stairsYaw, 0.0, 0.0;  // [x y z yaw pitch roll]
    mpcRobotModel.setBasePose(state, basePose);
    mpcRobotModel.setJointAngles(state, referenceConfig.defaultJointState);

    // Average velocity toward the next knot (world frame) as the momentum/velocity target.
    if (k + 1 < knots.size()) {
      const scalar_t dt = std::max(knots[k + 1].time - knots[k].time, EPS);
      const vector3_t velocity = (knots[k + 1].position - knots[k].position) / dt;
      state(velocityIndex) = velocity(0);
      state(velocityIndex + 1) = velocity(1);
    }

    timeTrajectory.push_back(knots[k].time);
    stateTrajectory.push_back(std::move(state));
    inputTrajectory.push_back(vector_t::Zero(inputDim));
  }

  baseTargetTrajectories_ = TargetTrajectories(timeTrajectory, stateTrajectory, inputTrajectory);

  std::cout << "[StairClimbingPlan] compiled: " << swings.size() << " swings (" << footsteps_[LEFT_FOOT].size() << " left, "
            << footsteps_[RIGHT_FOOT].size() << " right), t in [" << startTime_ << ", " << finalTime_ << "], top height " << treadTopZ
            << " m" << std::endl;
}

/******************************************************************************************************/

scalar_t StairClimbingPlan::getPlannedFootHeight(size_t foot, scalar_t time) const {
  scalar_t height = config_.stairsBasePosition(2);
  for (const auto& footstep : footsteps_[foot]) {
    if (time >= footstep.touchDownTime - EPS) {
      height = footstep.touchDownPosition(2);
    } else {
      break;
    }
  }
  return height;
}

/******************************************************************************************************/

bool StairClimbingPlan::getSwingFootReference(size_t foot, scalar_t time, vector3_t& positionReference) const {
  // The xy reference reaches the foothold at this fraction of the swing and then
  // holds, so the foot has converged before the touch-down z constraint drives it
  // down (a reference that arrives exactly at touch-down leaves the tracking lag
  // uncorrected and the foot clips the stair nosing).
  constexpr scalar_t arrivalFraction = 0.75;
  for (const auto& footstep : footsteps_[foot]) {
    if (time >= footstep.liftOffTime && time <= footstep.touchDownTime) {
      const scalar_t duration = std::max(footstep.touchDownTime - footstep.liftOffTime, EPS);
      const scalar_t progress = std::clamp((time - footstep.liftOffTime) / (arrivalFraction * duration), 0.0, 1.0);
      positionReference = footstep.liftOffPosition + progress * (footstep.touchDownPosition - footstep.liftOffPosition);
      return true;
    }
  }
  return false;
}

/******************************************************************************************************/

void StairClimbingPlan::getHeightSequences(const ModeSchedule& modeSchedule,
                                           feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                           feet_array_t<scalar_array_t>& touchDownHeightSequence) const {
  const auto& eventTimes = modeSchedule.eventTimes;
  const size_t numPhases = modeSchedule.modeSequence.size();

  // Phase p is active in [eventTimes[p-1], eventTimes[p]] (open-ended at both ends).
  const auto phaseStartTime = [&](size_t p) -> scalar_t {
    return (p == 0) ? (eventTimes.empty() ? startTime_ : eventTimes.front() - 1.0) : eventTimes[p - 1];
  };
  const auto phaseEndTime = [&](size_t p) -> scalar_t {
    return (p >= eventTimes.size()) ? (eventTimes.empty() ? finalTime_ : eventTimes.back() + 1.0) : eventTimes[p];
  };

  for (size_t foot = 0; foot < liftOffHeightSequence.size(); ++foot) {
    liftOffHeightSequence[foot].resize(numPhases);
    touchDownHeightSequence[foot].resize(numPhases);
    for (size_t p = 0; p < numPhases; ++p) {
      // Lift-off height: the foot's support height when the phase begins (heights
      // change at touch-down events, so just after the phase-start event).
      liftOffHeightSequence[foot][p] = getPlannedFootHeight(foot, phaseStartTime(p) + 1e-4);
      // Touch-down height: the foot's support height right after the phase ends.
      touchDownHeightSequence[foot][p] = getPlannedFootHeight(foot, phaseEndTime(p) + 1e-4);
    }
  }
}

}  // namespace ocs2::humanoid
