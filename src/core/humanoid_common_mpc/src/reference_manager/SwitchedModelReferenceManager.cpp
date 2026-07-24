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
  const auto it = std::upper_bound(modeSchedule_.eventTimes.begin(), modeSchedule_.eventTimes.end(), time);
  scalar_t nextEventTime = *it;
  scalar_t prevEventTime = *(it - 1);

  if (modeSchedule_.modeAtTime(time) == LF) {
    return (0.5 * (time - prevEventTime) / (nextEventTime - prevEventTime));
  } else if (modeSchedule_.modeAtTime(time) == RF) {
    return (0.5 + 0.5 * (time - prevEventTime) / (nextEventTime - prevEventTime));
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

  terrainHeight = 0.0;

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

    // Interpret the commanded pelvis height as height-above-support (capped so
    // the rear leg can still reach during a tread transfer); zero pitch/roll.
    // The horizontal reference is clamped to a maximum lead over the planned
    // support midpoint, and the forward momentum reference is damped by that
    // same lead, so the base reference "arrives over the support and waits":
    // a velocity command cannot drag the CoM past where the feet can be placed
    // on the terrain (which otherwise topples the robot over the first riser).
    
    const size_t velStart = mpcRobotModelPtr_->getBaseComVelocityStartindex();
    for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); i++) {
      vector_t& targetState = targetTrajectories.stateTrajectory[i];
      const scalar_t knotTime = targetTrajectories.timeTrajectory[i];
      vector6_t basePose = mpcRobotModelPtr_->getBasePose(targetState);

      // Away from the stairs, leave the velocity-command base reference untouched
      // so the robot walks normally (base_twist behavior) on the flat approach;
      // the terrain adaptation below engages only near/on the stairs.
      if (!planner.isNearStairs(basePose.head<2>())) {
        continue;
      }

      // Anticipated foot position: during a swing, blend the previous foothold
      // toward the upcoming one by swing progress so the base reference moves
      // forward AND rises onto the step AS the foot does (using only the
      // committed touchdown lags the base behind/below the step and pitches the
      // robot backward off it). Committed positions are used for the lateral
      // weight-shift target (the actual stance foot).
      const auto anticipatedFoot = [&](size_t foot, scalar_t t) -> vector3_t {
        const auto& steps = planner.getFootsteps()[foot];
        vector3_t pos = steps.empty() ? planner.getPlannedFootPosition(foot, t) : steps.front().liftOffPosition;
        for (const auto& fs : steps) {
          if (t >= fs.touchDownTime) {
            pos = fs.touchDownPosition;
          } else if (t >= fs.liftOffTime) {
            const scalar_t prog = std::clamp((t - fs.liftOffTime) / std::max(fs.touchDownTime - fs.liftOffTime, 1e-6), 0.0, 1.0);
            pos = fs.liftOffPosition + prog * (fs.touchDownPosition - fs.liftOffPosition);
            break;
          } else {
            break;
          }
        }
        return pos;
      };

      const vector3_t footL = planner.getPlannedFootPosition(0, knotTime);  // committed (stance)
      const vector3_t footR = planner.getPlannedFootPosition(1, knotTime);
      const vector3_t supportMidpoint = 0.5 * (anticipatedFoot(0, knotTime) + anticipatedFoot(1, knotTime));
      const Eigen::Vector2d horizontalOffset = basePose.head<2>() - supportMidpoint.head<2>();
      const scalar_t maxLead = planner.getMaxBaseLead();

      // Decompose the base lead over the support into forward / lateral (yaw frame).
      const scalar_t yaw = basePose(3);
      const Eigen::Vector2d forwardDir(std::cos(yaw), std::sin(yaw));
      const Eigen::Vector2d lateralDir(-std::sin(yaw), std::cos(yaw));
      const scalar_t forwardLead = horizontalOffset.dot(forwardDir);

      // Damp the horizontal momentum/velocity reference as the base approaches
      // its maximum FORWARD lead. A deadzone at half the max lead keeps normal
      // (flat-ground) walking at full speed and only gates gross overrun, e.g.
      // when the feet are blocked by a riser: full speed below 0.5*maxLead,
      // ramping to zero at maxLead.
      const scalar_t velScale = std::clamp(2.0 * (maxLead - std::max(forwardLead, 0.0)) / maxLead, 0.0, 1.0);
      targetState(velStart + 0) *= velScale;
      targetState(velStart + 1) *= velScale;

      // Lateral weight shift: in single support the pelvis reference must move
      // over the stance foot (a biped cannot stay centered while stepping up
      // onto a tread). In double support target the support midpoint.
      const contact_flag_t contacts = modeNumber2StanceLeg(modeSchedule.modeAtTime(knotTime));
      // Fraction of the way to the stance foot the pelvis reference shifts in
      // single support. Full shift (0.8) overshot and, coupled with the rise
      // onto the step, rolled the robot; a partial shift leaves the momentum to
      // carry the CoM, like normal walking.
      constexpr scalar_t lateralShiftFraction = 0.6;
      scalar_t lateralTarget = 0.0;  // signed offset from support midpoint along lateralDir
      if (contacts[0] && !contacts[1]) {
        lateralTarget = lateralShiftFraction * (footL.head<2>() - supportMidpoint.head<2>()).dot(lateralDir);
      } else if (contacts[1] && !contacts[0]) {
        lateralTarget = lateralShiftFraction * (footR.head<2>() - supportMidpoint.head<2>()).dot(lateralDir);
      }

      const scalar_t clampedForward = std::clamp(forwardLead, -maxLead, maxLead);
      basePose.head<2>() = supportMidpoint.head<2>() + clampedForward * forwardDir + lateralTarget * lateralDir;

      basePose(2) = supportMidpoint(2) + std::min(basePose(2), planner.getMaxBaseHeightAboveSupport());
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
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

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
  const auto& stairPlan = stairClimbingPlan_.get();
  if (stairPlan && stairPlan->getSwingFootReference(foot, time, positionReference)) {
    trackingWeight = stairPlan->getFootholdTrackingWeight();
    return true;
  }
  if (isTerrainWalkActive() && terrainFootholdPlannerPtr_->getSwingFootReference(foot, time, positionReference)) {
    trackingWeight = terrainFootholdPlannerPtr_->getFootholdTrackingWeight();
    return true;
  }
  return false;
}

}  // namespace ocs2::humanoid
