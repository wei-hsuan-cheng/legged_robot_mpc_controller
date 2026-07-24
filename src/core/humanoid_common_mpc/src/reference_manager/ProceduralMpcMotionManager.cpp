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

#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"


#include <cmath>
#include "humanoid_common_mpc/gait/GaitScheduleUpdater.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ProceduralMpcMotionManager::ProceduralMpcMotionManager(GaitMap gaitMap,
                                                       const ReferenceConfig& referenceConfig,
                                                       std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
                                                       const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                       VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories,
                                                       BasePoseTargetToTargetTrajectories basePoseTargetToTargetTrajectories)
    : switchedModelReferenceManagerPtr_(switchedModelReferenceManagerPtr),
      gaitSchedulePtr_(switchedModelReferenceManagerPtr_->getGaitSchedule()),
      mpcRobotModelPtr_(&mpcRobotModel),
      walkingVelocityTarget_(referenceConfig, std::move(velocityTargetToTargetTrajectories)),
      basePoseTarget_(referenceConfig, mpcRobotModel, std::move(basePoseTargetToTargetTrajectories)),
      referenceConfig_(referenceConfig) {
  gaitMap_ = std::move(gaitMap);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::setStairClimbingConfig(StairClimbingConfig config) {
  std::lock_guard<std::mutex> lock(stairClimbingConfigMutex_);
  stairClimbingConfig_ = std::make_shared<const StairClimbingConfig>(std::move(config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::setVelocityCommand(const WalkingVelocityCommand& command) {
  walkingVelocityTarget_.setCommand(command);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::setBasePoseCommand(const BasePoseCommand& command) {
  basePoseTarget_.setCommand(command);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::setTargetMode(TargetMode mode) {
  const TargetMode previousMode = targetMode_.load(std::memory_order_acquire);
  if (mode == TargetMode::BasePose && previousMode != TargetMode::BasePose) {
    basePoseTarget_.requestCurrentPoseLatch();
  }
  targetMode_.store(mode, std::memory_order_release);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ProceduralMpcMotionManager::transitionToFasterGait(const vector4_t& velCommandVec,
                                                        const vector6_t& baseVelocity,
                                                        const GaitModeStateConfig& cfg) {
  bool fasterGaitRequested = (std::abs(velCommandVec(0)) > cfg.maxLinVelCmd || std::abs(velCommandVec(1)) > cfg.maxLinVelCmd ||
                              std::abs(velCommandVec(3)) > cfg.maxAngVelCmd);

  bool withinMaxSpeedErrorThreshold = (std::abs(baseVelocity(0)) > cfg.maxLinVelCmd - cfg.linVelErrorThresh ||
                                       std::abs(baseVelocity(1)) > cfg.maxLinVelCmd - cfg.linVelErrorThresh ||
                                       std::abs(baseVelocity(3)) > cfg.maxAngVelCmd - cfg.angVelErrorThresh);
  return fasterGaitRequested && withinMaxSpeedErrorThreshold;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ProceduralMpcMotionManager::transitionToSlowerGait(const vector4_t& velCommandVec,
                                                        const vector6_t& baseVelocity,
                                                        const GaitModeStateConfig& cfg) {
  bool slowerGaitRequested = (std::abs(velCommandVec(0)) < cfg.minLinVelCmd && std::abs(velCommandVec(1)) < cfg.minLinVelCmd &&
                              std::abs(velCommandVec(3)) < cfg.minAngVelCmd);

  bool baseSpeedSlowEnough = (std::abs(baseVelocity(0)) < cfg.minLinVelCmd + cfg.linVelErrorThresh &&
                              std::abs(baseVelocity(1)) < cfg.minLinVelCmd + cfg.linVelErrorThresh &&
                              std::abs(velCommandVec(3)) < cfg.minAngVelCmd + cfg.angVelErrorThresh);

  return slowerGaitRequested && baseSpeedSlowEnough;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::preSolverRun(scalar_t initTime,
                                              scalar_t finalTime,
                                              const vector_t& initState,
                                              const ReferenceManagerInterface& referenceManager) {
  if (getTargetMode() == TargetMode::StairClimb) {
    switchedModelReferenceManagerPtr_->setTerrainWalkActive(false);
    runStairClimbing(initTime, initState);
    return;
  }
  // Terrain-aware walking rides the velocity-command path below; the online
  // foothold planner in the reference manager does the terrain adaptation.
  switchedModelReferenceManagerPtr_->setTerrainWalkActive(getTargetMode() == TargetMode::TerrainWalk);
  if (activeStairClimbingPlan_) {
    // Left stair climbing mode: drop the plan so the swing planner reverts to
    // flat ground and the gait FSM takes over again from stance.
    activeStairClimbingPlan_.reset();
    switchedModelReferenceManagerPtr_->setStairClimbingPlan(nullptr);
    stairClimbFinishedLogged_ = false;
    currentGaitMode_ = 0;
    currentGaitCommand_ = lastGaitCommand_ = "stance";
    std::cout << "[ProceduralMpcMotionManager] Stair climbing plan cleared." << std::endl;
  }

  vector4_t filteredVelCommand;
  if (getTargetMode() == TargetMode::BasePose) {
    BasePoseTarget::Output target = basePoseTarget_.evaluate(initTime, finalTime, initState);
    switchedModelReferenceManagerPtr_->setTargetTrajectories(std::move(target.targetTrajectories));
    filteredVelCommand = target.motionCommand;
  } else {
    WalkingVelocityTarget::Output target = walkingVelocityTarget_.evaluate(initTime, finalTime, initState);
    switchedModelReferenceManagerPtr_->setTargetTrajectories(std::move(target.targetTrajectories));
    filteredVelCommand = target.conditionedCommand;
  }

  static GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
  vector6_t baseVelocity = mpcRobotModelPtr_->getBaseComVelocity(initState);

  // Terrain-aware walking never escalates beyond the "slow_walk" gait: faster
  // gaits shorten or drop the double support, which is unsafe on stairs.
  constexpr size_t slowWalkGaitModeIndex = 1;  // index of "slow_walk" in gaitModeStates_
  const size_t maxGaitMode =
      (getTargetMode() == TargetMode::TerrainWalk) ? slowWalkGaitModeIndex : gaitModeStates_.size() - 1;
  if (currentGaitMode_ > maxGaitMode) {
    currentGaitMode_ = maxGaitMode;
    currentCfg = gaitModeStates_[currentGaitMode_];
    currentGaitCommand_ = currentCfg.gaitCommand;
    lastGaitChangeTime_ = initTime;
  }

  // Do not change the gait pattern for at least 0.5s
  if (initTime > lastGaitChangeTime_ + 0.2) {
    if (currentGaitMode_ < maxGaitMode && transitionToFasterGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_++;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Increasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    } else if (transitionToSlowerGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_--;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Decreasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    }
  }

  if (currentGaitCommand_ != lastGaitCommand_) {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);

    GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
    lastGaitCommand_ = currentGaitCommand_;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::runStairClimbing(scalar_t initTime, const vector_t& initState) {
  if (!activeStairClimbingPlan_) {
    std::shared_ptr<const StairClimbingConfig> config;
    {
      std::lock_guard<std::mutex> lock(stairClimbingConfigMutex_);
      config = stairClimbingConfig_;
    }
    if (!config) {
      std::cerr << "[ProceduralMpcMotionManager] StairClimb mode selected but no stair climbing config is loaded." << std::endl;
      return;
    }

    // Anchor the plan at the current solver time, starting from the current base pose.
    const vector6_t basePose = mpcRobotModelPtr_->getBasePose(initState);
    auto plan = std::make_shared<const StairClimbingPlan>(*config, initTime, basePose, *mpcRobotModelPtr_, referenceConfig_);

    // Insert the full (non-periodic) climb sequence once, then park the stored
    // template on stance so the schedule extends with stance after the climb.
    gaitSchedulePtr_->insertModeSequenceTemplate(plan->getModeSequenceTemplate(), initTime, plan->getFinalTime() - 1e-3);
    gaitSchedulePtr_->insertModeSequenceTemplate(gaitMap_.at("stance"), plan->getFinalTime(), plan->getFinalTime() + 0.5);

    switchedModelReferenceManagerPtr_->setStairClimbingPlan(plan);
    activeStairClimbingPlan_ = std::move(plan);
    stairClimbFinishedLogged_ = false;

    // Park the velocity-gait FSM while the fixed sequence runs.
    currentGaitMode_ = 0;
    currentGaitCommand_ = lastGaitCommand_ = "stance";

    std::cout << "[ProceduralMpcMotionManager] Stair climbing started at t=" << initTime << ", finishes at t="
              << activeStairClimbingPlan_->getFinalTime() << std::endl;
  }

  // The plan's base reference covers the whole climb with absolute times; the
  // solver interpolates (and holds the final state after the climb ends).
  switchedModelReferenceManagerPtr_->setTargetTrajectories(TargetTrajectories(activeStairClimbingPlan_->getBaseTargetTrajectories()));

  if (!stairClimbFinishedLogged_ && initTime > activeStairClimbingPlan_->getFinalTime()) {
    std::cout << "[ProceduralMpcMotionManager] Stair climbing sequence finished; holding stance on the last step." << std::endl;
    stairClimbFinishedLogged_ = true;
  }
}

}  // namespace ocs2::humanoid
