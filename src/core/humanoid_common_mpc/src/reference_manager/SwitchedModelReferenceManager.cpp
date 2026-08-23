/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      // The reference manager gets a copy of the pinocchio model to use for initializing the ground height
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t SwitchedModelReferenceManager::getPhaseVariable(scalar_t time) const {
  // This feeds the arm-swing reference through getDesiredState(), so it runs on
  // every solver iteration. It used to dereference unconditionally: *it is out of
  // bounds when `time` is past the last event (it == end()), and *(it - 1) is out
  // of bounds when `time` precedes the first (it == begin()). Both happen in
  // normal operation, because the cost is evaluated across a horizon that can
  // extend beyond the schedule that was planned for it. The values read were
  // whatever happened to be adjacent in memory, which could make the phase
  // variable - and hence the commanded arm angles - jump arbitrarily.
  const auto& eventTimes = modeSchedule_.eventTimes;
  if (eventTimes.empty()) {
    return 0.0;
  }
  const auto it = std::upper_bound(eventTimes.begin(), eventTimes.end(), time);
  if (it == eventTimes.begin() || it == eventTimes.end()) {
    // Outside the planned schedule there is no meaningful phase; report the start
    // of the cycle rather than extrapolating from out-of-range samples.
    return 0.0;
  }
  const scalar_t nextEventTime = *it;
  const scalar_t prevEventTime = *(it - 1);
  const scalar_t cycleDuration = nextEventTime - prevEventTime;
  if (!(cycleDuration > 0.0)) {
    return 0.0;  // coincident events: no phase to interpolate, and no division
  }

  if (modeSchedule_.modeAtTime(time) == LF) {
    return (0.5 * (time - prevEventTime) / cycleDuration);
  } else if (modeSchedule_.modeAtTime(time) == RF) {
    return (0.5 + 0.5 * (time - prevEventTime) / cycleDuration);
  } else {
    if (modeSchedule_.modeAtTime(prevEventTime - 0.01) == LF) {
      return 0.5;
    } else {
      return 0;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelReferenceManager::adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories,
                                                                   const vector_t& initState,
                                                                   size_t initMode) {
  scalar_t terrainHeight = computeGroundHeightEstimate(pinocchioInterface_, *mpcRobotModelPtr_,
                                                       mpcRobotModelPtr_->getGeneralizedCoordinates(initState), initMode);

  // This value is the swing planner's ground reference and the base-height target
  // offset, so pinning it to zero hard-codes a flat floor at the world origin -
  // the estimate above was computed and then thrown away. That is the single
  // thing preventing the stack from walking on sloped or stepped ground, no
  // matter how well the state estimator tracks the terrain: the swing foot would
  // still be aimed at z = 0.
  //
  // Off by default because every gain in this configuration was tuned against the
  // flat assumption, and because the estimate is only as good as the foot heights
  // the estimator supplies - on flat ground it is strictly worse than the
  // constant it replaces. Turn it on together with an InEKF height source that
  // does not itself assume a flat plane ("inekf" or "anchored"; "kinematic" and
  // "blend" both pin the feet to groundZ and would defeat the point).
  if (!useTerrainHeightEstimate_) {
    terrainHeight = 0.0;
  } else if (!std::isfinite(terrainHeight)) {
    terrainHeight = previousGroundHeightEstimate_;  // never feed a NaN to the planner
  } else {
    // Rate-limit per solve: a mis-detected contact can otherwise step the
    // reference by most of a leg length in one iteration, which the swing planner
    // turns into an impossible foot trajectory.
    terrainHeight = std::clamp(terrainHeight,
                               previousGroundHeightEstimate_ - maxTerrainHeightStep_,
                               previousGroundHeightEstimate_ + maxTerrainHeightStep_);
  }

  // adapt target Trajectories to current terrain height
  // Since they are published in the past the current observations ground height might have drifted.

  // Adapt the ground height difference for every state in the target Trajectories.
  // The height difference between last update and the current update is applied here
  // to prevent applying the same difference twice in case the trajectories have not been updated.
  for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); i++) {
    vector_t& targetState = targetTrajectories.stateTrajectory[i];
    scalar_t heightDifference = terrainHeight - previousGroundHeightEstimate_;
    mpcRobotModelPtr_->adaptBasePoseHeight(targetState, heightDifference);
  }
  previousGroundHeightEstimate_ = terrainHeight;
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t SwitchedModelReferenceManager::getDesiredState(const TargetTrajectories& targetTrajectories,
                                                        const vector_t& state,
                                                        scalar_t time) const {
  vector_t xNominal = targetTrajectories.getDesiredState(time);

  if (armSwingReferenceActive_) {
    scalar_t phaseVariable = this->getPhaseVariable(time);
    vector_t desiredJointAngles = mpcRobotModelPtr_->getJointAngles(xNominal);

    vector3_t linVelCommand = mpcRobotModelPtr_->getBaseComLinearVelocity(xNominal);
    scalar_t currentEulerZ = mpcRobotModelPtr_->getBasePose(state)[3];

    const scalar_t localVelXCommand = (std::cos(currentEulerZ) * linVelCommand[0] + std::sin(currentEulerZ) * linVelCommand[1]);

    const ModelSettings& modelSettings = mpcRobotModelPtr_->modelSettings;

    scalar_t gaitCycleFactor = std::sin(2 * M_PI * (phaseVariable - 0.15)) * localVelXCommand;
    desiredJointAngles[modelSettings.j_l_shoulder_y_index] += -0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_r_shoulder_y_index] += 0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_l_elbow_y_index] += -0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_r_elbow_y_index] += 0.15 * gaitCycleFactor;

    mpcRobotModelPtr_->setJointAngles(xNominal, desiredJointAngles);
  }
  return xNominal;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime,
                                                     scalar_t finalTime,
                                                     const vector_t& initState,
                                                     size_t initMode,
                                                     TargetTrajectories& targetTrajectories,
                                                     ModeSchedule& modeSchedule) {
  // Swap in the latest external targets on the solver thread before the costs read them.
  externalJointTargets_.updateFromBuffer();
  externalFrameRelationTargets_.updateFromBuffer();
  stairClimbingPlan_.updateFromBuffer();
  terrainWalkActive_.updateFromBuffer();
  baseTrackingMode_.updateFromBuffer();

  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  const auto& stairPlan = stairClimbingPlan_.get();
  if (stairPlan) {
    // Fixed-sequence stair climbing: feed the planned per-phase support heights
    // to the swing planner instead of the flat-ground estimate.
    feet_array_t<scalar_array_t> liftOffHeightSequence;
    feet_array_t<scalar_array_t> touchDownHeightSequence;
    stairPlan->getHeightSequences(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
  } else if (isTerrainWalkActive()) {
    // Terrain-aware walking: re-plan the footsteps over the ground-truth
    // terrain for the current window, terrain-adapt the base height reference,
    // and feed the planned support heights to the swing planner.
    auto& planner = *terrainFootholdPlannerPtr_;
    planner.update(modeSchedule, targetTrajectories, computeFeetPositions(initState), initState, initTime, *mpcRobotModelPtr_);

    // Keep the velocity-command trajectory on the flat approach. Near the
    // terrain, constrain its lead over the planned support and adapt its height
    // using committed swing footholds. This preserves persistent forward intent
    // instead of rebuilding the reference origin from measured feet every solve.
    const size_t velStart = mpcRobotModelPtr_->getBaseComVelocityStartindex();
    for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); ++i) {
      vector_t& targetState = targetTrajectories.stateTrajectory[i];
      const scalar_t knotTime = targetTrajectories.timeTrajectory[i];
      vector6_t basePose = mpcRobotModelPtr_->getBasePose(targetState);

      if (!planner.isNearStairs(basePose.head<2>())) {
        continue;
      }

      const auto anticipatedFoot = [&](size_t foot, scalar_t time) -> vector3_t {
        vector3_t position = planner.getPlannedFootPosition(foot, time);
        for (const auto& footstep : planner.getFootsteps()[foot]) {
          if (time >= footstep.touchDownTime) {
            position = footstep.touchDownPosition;
          } else if (time >= footstep.liftOffTime) {
            const scalar_t duration = std::max(footstep.touchDownTime - footstep.liftOffTime, 1e-6);
            const scalar_t progress = std::clamp((time - footstep.liftOffTime) / duration, 0.0, 1.0);
            position = footstep.liftOffPosition + progress * (footstep.touchDownPosition - footstep.liftOffPosition);
            break;
          } else {
            break;
          }
        }
        return position;
      };

      const vector3_t supportMidpoint =
          0.5 * (anticipatedFoot(0, knotTime) + anticipatedFoot(1, knotTime));
      const scalar_t yaw = basePose(3);
      const Eigen::Vector2d forwardDirection(std::cos(yaw), std::sin(yaw));
      const Eigen::Vector2d lateralDirection(-std::sin(yaw), std::cos(yaw));
      const Eigen::Vector2d horizontalOffset = basePose.head<2>() - supportMidpoint.head<2>();
      const scalar_t forwardLead = horizontalOffset.dot(forwardDirection);
      const scalar_t lateralLead = horizontalOffset.dot(lateralDirection);
      const scalar_t maxLead = std::max(planner.getMaxBaseLead(), 1e-6);

      const scalar_t velocityScale =
          std::clamp(2.0 * (maxLead - std::max(forwardLead, 0.0)) / maxLead, 0.0, 1.0);
      targetState(velStart) *= velocityScale;
      targetState(velStart + 1) *= velocityScale;

      basePose.head<2>() =
          supportMidpoint.head<2>() +
          std::clamp(forwardLead, -maxLead, maxLead) * forwardDirection +
          std::clamp(lateralLead, -maxLead, maxLead) * lateralDirection;
      basePose(2) =
          supportMidpoint(2) + std::min(basePose(2), planner.getMaxBaseHeightAboveSupport());
      basePose(4) = 0.0;
      basePose(5) = 0.0;
      mpcRobotModelPtr_->setBasePose(targetState, basePose);
    }

    feet_array_t<scalar_array_t> liftOffHeightSequence;
    feet_array_t<scalar_array_t> touchDownHeightSequence;
    planner.getHeightSequences(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
  } else {
    scalar_t terrainHeight = adaptToCurrentGroundHeight(targetTrajectories, initState, initMode);
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
  }

  modeSchedule_ = modeSchedule;

  // Recompute the flat-ground capture-point footholds from the state this solve
  // starts at, so the swing target reflects the CURRENT lateral velocity rather
  // than a fixed nominal step.
  updateCaptureFootholds(initTime, initState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void SwitchedModelReferenceManager::updateCaptureFootholds(scalar_t initTime, const vector_t& initState) {
  captureFootholdValid_ = {false, false};
  if (!captureFootPlacement_.enabled) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(initState);
  pinocchio::centerOfMass(model, data, q, false);

  const vector2_t com = data.com[0].head<2>();
  const scalar_t comHeight = std::max(data.com[0](2), 0.1);
  // For the centroidal model the normalized linear momentum IS the CoM velocity.
  const vector2_t comVelocity = mpcRobotModelPtr_->getBaseComLinearVelocity(initState).head<2>();

  // omega = sqrt(g / z_com); the capture point is com + v/omega, i.e. where the
  // CoM would come to rest if the foot were placed there.
  const scalar_t omega = std::sqrt(9.81 / comHeight);
  const vector2_t capturePoint = com + comVelocity / omega;

  const vector6_t basePose = mpcRobotModelPtr_->getBasePose(initState);
  const scalar_t yaw = basePose(3);
  const vector2_t lateralDirection(-std::sin(yaw), std::cos(yaw));

  const contact_flag_t contactFlags = getContactFlags(initTime);
  const scalar_t halfWidth = 0.5 * captureFootPlacement_.stepWidth;
  const feet_array_t<vector3_t> feetPositions = computeFeetPositions(initState);

  // Time remaining until the next contact switch, i.e. until the swinging foot
  // lands. The DCM diverges from the stance foot as exp(omega * t), so a
  // correction built from the CURRENT capture point is systematically too small
  // by that factor - which shows up as a roll that still grows, only slower.
  // Propagating the DCM to touchdown is what makes the placement deadbeat.
  scalar_t timeToTouchdown = 0.0;
  {
    const auto& eventTimes = modeSchedule_.eventTimes;
    const auto it = std::upper_bound(eventTimes.begin(), eventTimes.end(), initTime);
    if (it != eventTimes.end()) {
      timeToTouchdown = std::max(*it - initTime, 0.0);
    }
    timeToTouchdown = std::min(timeToTouchdown, 1.0);  // guard against a stale schedule
  }

  // Step period, from the spacing of the scheduled contact switches. Used for the
  // feedforward step length below; a walking robot must land its foot AHEAD of
  // the body, which the capture point on its own never asks for.
  scalar_t stepPeriod = 0.0;
  {
    const auto& eventTimes = modeSchedule_.eventTimes;
    const auto it = std::upper_bound(eventTimes.begin(), eventTimes.end(), initTime);
    if (it != eventTimes.end() && std::next(it) != eventTimes.end()) {
      stepPeriod = std::clamp(*std::next(it) - *it, 0.0, 2.0);
    } else if (it != eventTimes.end() && it != eventTimes.begin()) {
      stepPeriod = std::clamp(*it - *std::prev(it), 0.0, 2.0);
    }
  }
  // Bound the DCM extrapolation separately: exp() of a long horizon turns a small
  // velocity error into an unreachable foothold, and the clamp below would then
  // saturate every step and destroy the feedback. Measured: a 0.4 s horizon is
  // worse than none at all.
  const scalar_t dcmHorizon = std::min(timeToTouchdown, captureFootPlacement_.projectionHorizon);

  for (size_t foot = 0; foot < captureFoothold_.size(); ++foot) {
    if (contactFlags[foot]) {
      continue;  // a foot already on the ground is not being placed
    }
    // Foot 0 is the left foot, so it sits on the positive lateral side.
    const scalar_t side = (foot == 0) ? 1.0 : -1.0;

    // The nominal foothold has to sit under where the BASE WILL BE when the foot
    // lands, not under where it is now. Using the current base position biases
    // every foothold backwards by v * timeToTouchdown - about 0.08 m at 0.2 m/s
    // over a 0.4 s swing - so the robot is perpetually stepping short and pitches
    // forward, and the error grows with speed. That matches the measured failure:
    // pitch creeping up through a walk and diverging sooner the faster it goes.
    const vector2_t baseAtTouchdown = basePose.head<2>() + comVelocity * timeToTouchdown;

    // Feedforward step length: the foot has to land ahead of the base so the body
    // can pass over it, which the capture point (a stopping placement) never
    // asks for. See CaptureFootPlacementSettings::stepLengthGain.
    const vector2_t stepAhead =
        captureFootPlacement_.stepLengthGain * stepPeriod * comVelocity;

    const vector2_t nominal =
        baseAtTouchdown + stepAhead + side * halfWidth * lateralDirection;

    // Propagate the capture point about the supporting foot to the moment this
    // foot lands: xi_td = p_stance + (xi_now - p_stance) * exp(omega * dt).
    const size_t stanceFoot = 1 - foot;
    const vector2_t stancePosition = feetPositions[stanceFoot].head<2>();
    const scalar_t growth = std::exp(omega * dcmHorizon);
    const vector2_t capturePointAtTouchdown =
        stancePosition + (capturePoint - stancePosition) * growth;

    // The correction is what turns a fixed step into feedback: how far the
    // predicted capture point sits from where the base will be when the foot
    // lands. Measured against the same predicted base as the nominal, so the two
    // do not double-count the forward travel.
    // Clamped so a transient cannot command a step the leg cannot reach.
    vector2_t correction =
        captureFootPlacement_.gain * (capturePointAtTouchdown - baseAtTouchdown);
    const scalar_t limit = captureFootPlacement_.maxAdjustment;
    correction.x() = std::clamp(correction.x(), -limit, limit);
    correction.y() = std::clamp(correction.y(), -limit, limit);

    captureFoothold_[foot].head<2>() = nominal + correction;
    captureFoothold_[foot](2) = 0.0;  // flat ground
    captureFootholdValid_[foot] = true;
  }
}

feet_array_t<vector3_t> SwitchedModelReferenceManager::computeFeetPositions(const vector_t& initState) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(initState);
  pinocchio::forwardKinematics(model, data, q);

  feet_array_t<vector3_t> feetPositions;
  const auto& contactNames = mpcRobotModelPtr_->modelSettings.contactNames;
  for (size_t foot = 0; foot < feetPositions.size(); ++foot) {
    const auto frameId = model.getFrameId(contactNames[foot]);
    feetPositions[foot] = pinocchio::updateFramePlacement(model, data, frameId).translation();
  }
  return feetPositions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool SwitchedModelReferenceManager::getSwingFootholdReference(size_t foot,
                                                              scalar_t time,
                                                              vector3_t& positionReference,
                                                              scalar_t& trackingWeight) const {
  // Terrain-aware plans win: they encode real geometry, whereas the capture-point
  // foothold below assumes flat ground.
  const auto& stairPlanForFoothold = stairClimbingPlan_.get();
  if (!stairPlanForFoothold && !isTerrainWalkActive() && captureFootPlacement_.enabled &&
      foot < captureFootholdValid_.size() && captureFootholdValid_[foot]) {
    positionReference = captureFoothold_[foot];
    trackingWeight = captureFootPlacement_.trackingWeight;
    return true;
  }

  const auto& stairPlan = stairClimbingPlan_.get();
  if (stairPlan && stairPlan->getSwingFootReference(foot, time, positionReference)) {
    trackingWeight = stairPlan->getFootholdTrackingWeight();
    return true;
  }
  if (isTerrainWalkActive() && terrainFootholdPlannerPtr_->getSwingFootReference(foot, time, positionReference)) {
    // Reach-scaled tracking weight: a foothold that is far ahead (an up-step the
    // foot cannot reach yet) gets a WEAK weight so the swing foot emerges from
    // base motion and the base keeps advancing (strong tracking of a far
    // foothold pins the feet and stalls the robot at the riser). As the base
    // approaches and the swing reach shrinks, the weight ramps to full so the
    // foot commits precisely onto the tread.
    scalar_t weight = terrainFootholdPlannerPtr_->getFootholdTrackingWeight();
    for (const auto& fs : terrainFootholdPlannerPtr_->getFootsteps()[foot]) {
      if (time >= fs.liftOffTime && time <= fs.touchDownTime) {
        const scalar_t reach = (fs.touchDownPosition.head<2>() - fs.liftOffPosition.head<2>()).norm();
        constexpr scalar_t commitReach = 0.20;  // reach at/below which the foot can commit -> full weight
        constexpr scalar_t farReach = 0.34;      // reach at/above which the foot cannot reach -> min weight
        constexpr scalar_t minScale = 0.05;
        weight *= std::clamp((farReach - reach) / (farReach - commitReach), minScale, 1.0);
        break;
      }
    }
    trackingWeight = weight;
    return true;
  }
  return false;
}

}  // namespace ocs2::humanoid
