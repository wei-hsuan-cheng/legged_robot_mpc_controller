/******************************************************************************
Copyright (c) 2026. All rights reserved.

Online foothold planner over ground-truth terrain, see TerrainFootholdPlanner.h.
******************************************************************************/

#include "humanoid_common_mpc/reference_manager/TerrainFootholdPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"

namespace ocs2::humanoid {

namespace {
constexpr scalar_t EPS = 1e-6;

vector2_t toLocal(const vector2_t& positionWorld, const vector2_t& center, scalar_t yaw) {
  const scalar_t c = std::cos(yaw);
  const scalar_t s = std::sin(yaw);
  const vector2_t d = positionWorld - center;
  return vector2_t(c * d(0) + s * d(1), -s * d(0) + c * d(1));
}

vector2_t toWorld(const vector2_t& positionLocal, const vector2_t& center, scalar_t yaw) {
  const scalar_t c = std::cos(yaw);
  const scalar_t s = std::sin(yaw);
  return center + vector2_t(c * positionLocal(0) - s * positionLocal(1), s * positionLocal(0) + c * positionLocal(1));
}
}  // namespace

/******************************************************************************************************/

scalar_t GroundTruthTerrainModel::heightAt(const vector2_t& positionWorld) const {
  scalar_t height = groundHeight;
  for (const auto& region : regions) {
    const vector2_t local = toLocal(positionWorld, region.center, region.yaw);
    if (std::abs(local(0)) <= region.halfLengthX + EPS && std::abs(local(1)) <= region.halfLengthY + EPS) {
      height = std::max(height, region.height);
    }
  }
  return height;
}

/******************************************************************************************************/

GroundTruthTerrainModel buildTerrainModelFromStairs(const StairClimbingConfig& stairsConfig) {
  GroundTruthTerrainModel terrain;
  terrain.groundHeight = stairsConfig.stairsBasePosition(2);

  const vector2_t base(stairsConfig.stairsBasePosition(0), stairsConfig.stairsBasePosition(1));
  const scalar_t yaw = stairsConfig.stairsYaw;

  scalar_t treadStartX = stairsConfig.startOffset;
  scalar_t treadTopZ = terrain.groundHeight;
  scalar_t totalDepth = 0.0;
  for (size_t i = 0; i < stairsConfig.stepHeights.size(); ++i) {
    const scalar_t depth = stairsConfig.stepDepths[i];
    treadTopZ += stairsConfig.stepHeights[i];

    PlanarRegion region;
    region.yaw = yaw;
    region.halfLengthX = 0.5 * depth;
    region.halfLengthY = 0.5 * stairsConfig.stepWidth;
    region.height = treadTopZ;
    region.center = toWorld(vector2_t(treadStartX + 0.5 * depth, 0.0), base, yaw);
    terrain.regions.push_back(region);

    treadStartX += depth;
    totalDepth += depth;
  }

  // Ground candidates are excluded from the staircase footprint.
  terrain.footprintYaw = yaw;
  terrain.footprintHalfLengthX = 0.5 * totalDepth;
  terrain.footprintHalfLengthY = 0.5 * stairsConfig.stepWidth;
  terrain.footprintCenter = toWorld(vector2_t(stairsConfig.startOffset + 0.5 * totalDepth, 0.0), base, yaw);

  return terrain;
}

/******************************************************************************************************/

vector3_t TerrainFootholdPlanner::projectFoothold(const vector2_t& nominalXY, scalar_t previousFootholdHeight) const {
  scalar_t bestScore = std::numeric_limits<scalar_t>::max();
  vector2_t bestXY = nominalXY;
  scalar_t bestHeight = terrainModel_.heightAt(nominalXY);
  bool haveCandidate = false;

  // A candidate is scored by its horizontal distance to the nominal foothold,
  // minus a bonus. Elevated steps get a step-up bonus so a reachable tread just
  // ahead beats the ground directly in front of the riser (otherwise the ground
  // candidate always wins and the robot flickers at the first step).
  const auto considerCandidate = [&](const vector2_t& candidateXY, scalar_t candidateHeight, scalar_t bonus) {
    if (std::abs(candidateHeight - previousFootholdHeight) > settings_.maxStepHeight) {
      return;  // unreachable in one step
    }
    const scalar_t score = (candidateXY - nominalXY).norm() - bonus;
    if (score < bestScore) {
      bestScore = score;
      bestXY = candidateXY;
      bestHeight = candidateHeight;
      haveCandidate = true;
    }
  };
  constexpr scalar_t stepUpBonus = 0.15;

  // Is the nominal inside the staircase footprint? If so, the flat ground does
  // not physically exist there (a step occupies that column), so a ground
  // candidate is only offered outside the footprint.
  bool insideFootprint = false;
  if (terrainModel_.footprintHalfLengthX > 0.0) {
    const vector2_t local = toLocal(nominalXY, terrainModel_.footprintCenter, terrainModel_.footprintYaw);
    insideFootprint = std::abs(local(0)) <= terrainModel_.footprintHalfLengthX &&
                      std::abs(local(1)) <= terrainModel_.footprintHalfLengthY;
  }
  if (!insideFootprint) {
    considerCandidate(nominalXY, terrainModel_.groundHeight, 0.0);
  }

  // Elevated regions: clamp the nominal into the margin-shrunk rectangle for
  // edge safety. A region is a candidate when the nominal lies within it (plus a
  // small slack, so a foot approaching a tread commits to stepping up onto it).
  constexpr scalar_t containmentSlack = 0.05;
  for (const auto& region : terrainModel_.regions) {
    const scalar_t usableX = region.halfLengthX - settings_.footMarginX;
    const scalar_t usableY = region.halfLengthY - settings_.footMarginY;
    if (usableX < 0.0 || usableY < 0.0) {
      continue;  // region too small for the foot
    }
    const vector2_t local = toLocal(nominalXY, region.center, region.yaw);
    if (std::abs(local(0)) > region.halfLengthX + containmentSlack ||
        std::abs(local(1)) > region.halfLengthY + containmentSlack) {
      continue;  // nominal not over this tread
    }
    const vector2_t clampedLocal(std::clamp(local(0), -usableX, usableX), std::clamp(local(1), -usableY, usableY));
    // Only steps ABOVE the previous foothold get the step-up bonus (do not bias
    // toward stepping down off the staircase).
    const scalar_t bonus = (region.height > previousFootholdHeight + 1e-3) ? stepUpBonus : 0.0;
    considerCandidate(toWorld(clampedLocal, region.center, region.yaw), region.height, bonus);
  }

  // Fallback: no reachable candidate (e.g. every option exceeds max_step_height)
  // -> step in place at the nominal, at whatever terrain is directly below it.
  if (!haveCandidate) {
    return vector3_t(nominalXY(0), nominalXY(1), terrainModel_.heightAt(nominalXY));
  }
  return vector3_t(bestXY(0), bestXY(1), bestHeight);
}

/******************************************************************************************************/

void TerrainFootholdPlanner::update(const ModeSchedule& modeSchedule,
                                    const TargetTrajectories& targetTrajectories,
                                    const feet_array_t<vector3_t>& currentFeetPositions,
                                    const vector_t& initState,
                                    scalar_t initTime,
                                    const MpcRobotModelBase<scalar_t>& mpcRobotModel) {
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& modeSequence = modeSchedule.modeSequence;
  const size_t numPhases = modeSequence.size();

  const auto phaseStartTime = [&](size_t p) -> scalar_t {
    return (p == 0) ? initTime - 1e3 : eventTimes[p - 1];
  };
  const auto phaseEndTime = [&](size_t p) -> scalar_t {
    return (p >= eventTimes.size()) ? initTime + 1e3 : eventTimes[p];
  };

  // The nominal footholds are extrapolated RELATIVE to the measured base pose:
  // current base + the desired displacement between now and touch-down. Using
  // the absolute desired positions instead couples any base tracking error
  // (e.g. lateral drift) into the footholds and collapses the stance width.
  const vector6_t currentBasePose = mpcRobotModel.getBasePose(initState);
  const vector_t desiredStateNow = targetTrajectories.getDesiredState(initTime);
  const vector6_t desiredPoseNow = mpcRobotModel.getBasePose(desiredStateNow);

  // Capture-point style feedback on the first upcoming step (paper eq. (12)):
  // offset the nominal foothold by sqrt(h/g) * (v_measured - v_desired).
  const vector3_t measuredVelocity = mpcRobotModel.getBaseComLinearVelocity(initState);
  const vector3_t desiredVelocityNow = mpcRobotModel.getBaseComLinearVelocity(desiredStateNow);
  vector2_t capturePointOffset = settings_.capturePointFeedbackGain *
                                 (measuredVelocity.head<2>() - desiredVelocityNow.head<2>());
  const scalar_t offsetNorm = capturePointOffset.norm();
  if (offsetNorm > settings_.maxCapturePointOffset) {
    capturePointOffset *= settings_.maxCapturePointOffset / offsetNorm;
  }

  for (size_t foot = 0; foot < footsteps_.size(); ++foot) {
    footsteps_[foot].clear();

    // The stance position before the first planned touch-down: measured foot,
    // snapped down to the terrain surface below it.
    initialFootPosition_[foot] = currentFeetPositions[foot];
    initialFootPosition_[foot](2) = terrainModel_.heightAt(currentFeetPositions[foot].head<2>());

    vector3_t previousFoothold = initialFootPosition_[foot];
    bool firstStep = true;

    for (size_t p = 0; p < numPhases; ++p) {
      const bool inContact = modeNumber2StanceLeg(modeSequence[p])[foot];
      if (inContact) {
        continue;
      }
      const scalar_t liftOffTime = phaseStartTime(p);
      const scalar_t touchDownTime = phaseEndTime(p);
      if (touchDownTime < initTime) {
        continue;  // swing fully in the past
      }

      // Nominal foothold: below the hip at touch-down (relative extrapolation),
      // plus a forward lead along the walking direction. The forward lead makes
      // the foothold reach into an upcoming tread's footprint so the planner
      // commits to stepping UP onto it (otherwise the ground candidate directly
      // below the hip always wins and the robot stalls at the first riser).
      const vector_t desiredState = targetTrajectories.getDesiredState(touchDownTime);
      const vector6_t desiredBasePose = mpcRobotModel.getBasePose(desiredState);
      const vector2_t baseXY = currentBasePose.head<2>() + (desiredBasePose.head<2>() - desiredPoseNow.head<2>());
      const scalar_t yaw = currentBasePose(3) + (desiredBasePose(3) - desiredPoseNow(3));
      const vector2_t forwardDir(std::cos(yaw), std::sin(yaw));
      const scalar_t lateral = (foot == 0) ? settings_.hipLateralOffset : -settings_.hipLateralOffset;
      constexpr scalar_t footholdForwardBias = 0.06;  // TODO promote to a config parameter
      vector2_t nominalXY = baseXY + footholdForwardBias * forwardDir +
                            vector2_t(-std::sin(yaw) * lateral, std::cos(yaw) * lateral);
      if (firstStep) {
        nominalXY += capturePointOffset;
        firstStep = false;
      }

      // Lateral alignment to the staircase centerline (ground truth): near/on the
      // stairs, anchor the stance laterally to the terrain axis instead of the
      // (veer-prone) base, so the biped keeps a symmetric stance while climbing.
      if (terrainModel_.footprintHalfLengthX > 0.0) {
        vector2_t local = toLocal(nominalXY, terrainModel_.footprintCenter, terrainModel_.footprintYaw);
        constexpr scalar_t approachDist = 0.5;
        if (local(0) > -terrainModel_.footprintHalfLengthX - approachDist) {  // approaching or on the stairs
          local(1) = (foot == 0) ? settings_.hipLateralOffset : -settings_.hipLateralOffset;
          nominalXY = toWorld(local, terrainModel_.footprintCenter, terrainModel_.footprintYaw);
        }
      }

      PlannedFootstep footstep;
      footstep.liftOffTime = liftOffTime;
      footstep.touchDownTime = touchDownTime;
      footstep.liftOffPosition = previousFoothold;
      footstep.touchDownPosition = projectFoothold(nominalXY, previousFoothold(2));
      footsteps_[foot].push_back(footstep);

      previousFoothold = footstep.touchDownPosition;
    }
  }
}

/******************************************************************************************************/

scalar_t TerrainFootholdPlanner::getPlannedFootHeight(size_t foot, scalar_t time) const {
  return getPlannedFootPosition(foot, time)(2);
}

/******************************************************************************************************/

vector3_t TerrainFootholdPlanner::getPlannedFootPosition(size_t foot, scalar_t time) const {
  vector3_t position = initialFootPosition_[foot];
  for (const auto& footstep : footsteps_[foot]) {
    if (time >= footstep.touchDownTime - EPS) {
      position = footstep.touchDownPosition;
    } else {
      break;
    }
  }
  return position;
}

/******************************************************************************************************/

bool TerrainFootholdPlanner::getSwingFootReference(size_t foot, scalar_t time, vector3_t& positionReference) const {
  for (const auto& footstep : footsteps_[foot]) {
    if (time >= footstep.liftOffTime && time <= footstep.touchDownTime) {
      const scalar_t duration = std::max(footstep.touchDownTime - footstep.liftOffTime, EPS);
      const scalar_t progress =
          std::clamp((time - footstep.liftOffTime) / (settings_.swingReferenceArrivalFraction * duration), 0.0, 1.0);
      positionReference = footstep.liftOffPosition + progress * (footstep.touchDownPosition - footstep.liftOffPosition);
      return true;
    }
  }
  return false;
}

/******************************************************************************************************/

void TerrainFootholdPlanner::getHeightSequences(const ModeSchedule& modeSchedule,
                                                feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                                feet_array_t<scalar_array_t>& touchDownHeightSequence) const {
  const auto& eventTimes = modeSchedule.eventTimes;
  const size_t numPhases = modeSchedule.modeSequence.size();

  const auto phaseStartTime = [&](size_t p) -> scalar_t {
    return (p == 0) ? (eventTimes.empty() ? 0.0 : eventTimes.front() - 1.0) : eventTimes[p - 1];
  };
  const auto phaseEndTime = [&](size_t p) -> scalar_t {
    return (p >= eventTimes.size()) ? (eventTimes.empty() ? 0.0 : eventTimes.back() + 1.0) : eventTimes[p];
  };

  for (size_t foot = 0; foot < liftOffHeightSequence.size(); ++foot) {
    liftOffHeightSequence[foot].resize(numPhases);
    touchDownHeightSequence[foot].resize(numPhases);
    for (size_t p = 0; p < numPhases; ++p) {
      liftOffHeightSequence[foot][p] = getPlannedFootHeight(foot, phaseStartTime(p) + 1e-4);
      touchDownHeightSequence[foot][p] = getPlannedFootHeight(foot, phaseEndTime(p) + 1e-4);
    }
  }
}

}  // namespace ocs2::humanoid
