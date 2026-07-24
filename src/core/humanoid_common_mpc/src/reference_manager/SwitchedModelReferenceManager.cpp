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

    // Replace the base (pelvis) reference with an ABSOLUTE trajectory built from
    // the feedback-replanned footholds: base = mid-feet xy + mean-support+crouch
    // z, moving forward and up in sync with the feet (the working fixed-sequence
    // base motion). The velocity-relative reference lagged with the measured
    // base and never accumulated forward, stalling the CoM at the riser
    // (marching-in-place). The default joint reference is carried over from the
    // incoming (velocity-command) target so arm/joint tracking is unchanged.
    const vector_t defaultJoints = targetTrajectories.stateTrajectory.empty()
                                       ? vector_t()
                                       : mpcRobotModelPtr_->getJointAngles(targetTrajectories.stateTrajectory.front());
    targetTrajectories = planner.getBaseTargetTrajectories(initTime, finalTime, *mpcRobotModelPtr_, defaultJoints);

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
