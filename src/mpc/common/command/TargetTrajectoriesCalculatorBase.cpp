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

#include "legged_robot_mpc_controller/mpc/common/command/TargetTrajectoriesCalculatorBase.h"

#include <stdexcept>

#include <algorithm>  // For std::clamp


#include <cmath>
#include "legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

TargetTrajectoriesCalculatorBase::TargetTrajectoriesCalculatorBase(const ReferenceConfig& referenceConfig,
                                                                   const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                   scalar_t mpcHorizon)
    : mpcRobotModelPtr_(mpcRobotModel.clone()), mpcHorizon_(mpcHorizon) {
  if (referenceConfig.defaultJointState.size() != mpcRobotModel.getJointDim()) {
    throw std::invalid_argument("[TargetTrajectoriesCalculatorBase] defaultJointState size (" +
                                std::to_string(referenceConfig.defaultJointState.size()) + ") does not match the MPC joint dimension (" +
                                std::to_string(mpcRobotModel.getJointDim()) + ")");
  }
  targetJointState_ = referenceConfig.defaultJointState;
  if (referenceConfig.postureHeights.size() != referenceConfig.postureJointStates.size()) {
    throw std::invalid_argument("[TargetTrajectoriesCalculatorBase] postureHeights and postureJointStates differ in length");
  }
  for (size_t i = 0; i < referenceConfig.postureHeights.size(); ++i) {
    if (referenceConfig.postureJointStates[i].size() != mpcRobotModel.getJointDim() ||
        (i > 0 && !(referenceConfig.postureHeights[i] > referenceConfig.postureHeights[i - 1]))) {
      throw std::invalid_argument("[TargetTrajectoriesCalculatorBase] posture table needs strictly increasing heights and "
                                  "one joint state of the MPC joint dimension per height");
    }
  }
  if (!std::isfinite(referenceConfig.minBaseHeight) || !std::isfinite(referenceConfig.maxBaseHeight) ||
      !(referenceConfig.minBaseHeight > 0.0) || !(referenceConfig.minBaseHeight <= referenceConfig.defaultBaseHeight) ||
      !(referenceConfig.defaultBaseHeight <= referenceConfig.maxBaseHeight)) {
    throw std::invalid_argument("[TargetTrajectoriesCalculatorBase] needs 0 < minBaseHeight <= defaultBaseHeight <= maxBaseHeight");
  }
  if (!referenceConfig.postureHeights.empty() && (referenceConfig.minBaseHeight < referenceConfig.postureHeights.front() ||
                                                  referenceConfig.maxBaseHeight > referenceConfig.postureHeights.back())) {
    throw std::invalid_argument("[TargetTrajectoriesCalculatorBase] posture table heights must cover [minBaseHeight, maxBaseHeight]");
  }
  postureHeights_ = referenceConfig.postureHeights;
  postureJointStates_ = referenceConfig.postureJointStates;
  defaultBaseHeight_ = referenceConfig.defaultBaseHeight;
  minBaseHeight_ = referenceConfig.minBaseHeight;
  maxBaseHeight_ = referenceConfig.maxBaseHeight;
  targetRotationVelocity_ = referenceConfig.targetRotationVelocity;
  targetDisplacementVelocity_ = referenceConfig.targetDisplacementVelocity;
  maxDisplacementVelocityX_ = referenceConfig.maxDisplacementVelocityX;
  maxDisplacementVelocityY_ = referenceConfig.maxDisplacementVelocityY;
  maxRotationVelocity_ = referenceConfig.maxRotationVelocity;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void TargetTrajectoriesCalculatorBase::setTargetJointState(const vector_t targetJointState) {
  assert(targetJointState.size() == mpcRobotModelPtr_->getJointDim());
  targetJointState_ = targetJointState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t TargetTrajectoriesCalculatorBase::getPostureForHeight(scalar_t height) const {
  if (postureHeights_.empty()) {
    return targetJointState_;
  }
  const scalar_t h = std::clamp(height, postureHeights_.front(), postureHeights_.back());
  const auto upper = std::upper_bound(postureHeights_.begin(), postureHeights_.end(), h);
  if (upper == postureHeights_.begin()) {
    return postureJointStates_.front();
  }
  if (upper == postureHeights_.end()) {
    return postureJointStates_.back();
  }
  const size_t i = static_cast<size_t>(upper - postureHeights_.begin());
  const scalar_t alpha = (h - postureHeights_[i - 1]) / (postureHeights_[i] - postureHeights_[i - 1]);
  return (1.0 - alpha) * postureJointStates_[i - 1] + alpha * postureJointStates_[i];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::getDeltaBaseTarget(const vector4_t& commadLinePoseTarget,
                                                               const vector6_t& currentPoseTarget) const {
  vector_t target(6);

  // X facing forward to the robot, Y to the left side in the baseFrame
  const scalar_t baseFrameDeltaX = commadLinePoseTarget(0);
  const scalar_t baseFrameDeltaY = commadLinePoseTarget(1);
  const scalar_t currentEulerZ = currentPoseTarget(3);

  const scalar_t globalFrameDeltaX = std::cos(currentEulerZ) * baseFrameDeltaX - std::sin(currentEulerZ) * baseFrameDeltaY;
  const scalar_t globalFrameDeltaY = std::sin(currentEulerZ) * baseFrameDeltaX + std::cos(currentEulerZ) * baseFrameDeltaY;

  // base p_x, p_y are relative to current state
  target(0) = currentPoseTarget(0) + globalFrameDeltaX;
  target(1) = currentPoseTarget(1) + globalFrameDeltaY;
  // base z relative to the default height
  target(2) = std::clamp(defaultBaseHeight_ + commadLinePoseTarget(2), minBaseHeight_, maxBaseHeight_);
  // theta_z relative to current
  target(3) = currentPoseTarget(3) + commadLinePoseTarget(3) * M_PI / 180.0;
  target(4) = 0.0;
  target(5) = 0.0;

  return target;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::getCurrentBasePoseTarget(const vector_t& state) const {
  vector_t currentPoseTarget = mpcRobotModelPtr_->getBasePose(state);
  // Zero out roll and pitch of the torso since target trajectories starts from current state
  currentPoseTarget(4) = 0.0;
  currentPoseTarget(5) = 0.0;

  return currentPoseTarget;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector4_t TargetTrajectoriesCalculatorBase::transformVelCommandToGlobal(const vector4_t& commandedVelLocal,
                                                                        const scalar_t& currentEulerZ) const {
  vector4_t globalTargetVel = commandedVelLocal;

  globalTargetVel(0) = std::cos(currentEulerZ) * commandedVelLocal[0] - std::sin(currentEulerZ) * commandedVelLocal[1];
  globalTargetVel(1) = std::sin(currentEulerZ) * commandedVelLocal[0] + std::cos(currentEulerZ) * commandedVelLocal[1];

  return globalTargetVel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::integrateBodyTwistTargetBasePose(
    const vector6_t& currentPose, const vector4_t& commandedVelLocal, scalar_t deltaT) const {
  vector6_t targetPose = currentPose;

  const scalar_t vx = commandedVelLocal(0);
  const scalar_t vy = commandedVelLocal(1);
  const scalar_t yawRate = commandedVelLocal(3);
  const scalar_t deltaYaw = yawRate * deltaT;

  vector2_t displacementBody;
  if (std::abs(yawRate) < 1e-8) {
    displacementBody << vx * deltaT, vy * deltaT;
  } else {
    const scalar_t sinDelta = std::sin(deltaYaw);
    const scalar_t oneMinusCosDelta = 1.0 - std::cos(deltaYaw);
    displacementBody << (sinDelta * vx - oneMinusCosDelta * vy) / yawRate,
                        (oneMinusCosDelta * vx + sinDelta * vy) / yawRate;
  }

  const scalar_t cosYaw = std::cos(currentPose(3));
  const scalar_t sinYaw = std::sin(currentPose(3));
  targetPose(0) += cosYaw * displacementBody(0) - sinYaw * displacementBody(1);
  targetPose(1) += sinYaw * displacementBody(0) + cosYaw * displacementBody(1);
  targetPose(2) = commandedVelLocal(2);
  targetPose(3) += deltaYaw;
  targetPose[4] = 0.0;
  targetPose[5] = 0.0;
  return targetPose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t TargetTrajectoriesCalculatorBase::estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity_;
  return std::max(rotationTime, displacementTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t TargetTrajectoriesCalculatorBase::estimateTimeToBasePoseTarget(
    const vector6_t& desiredBaseDisplacement) const {
  const scalar_t translationTime = desiredBaseDisplacement.head<3>().norm() / targetDisplacementVelocity_;
  const scalar_t rotationTime = desiredBaseDisplacement.tail<3>().norm() / targetRotationVelocity_;
  return std::max({translationTime, rotationTime, scalar_t(0.1)});
}

}  // namespace ocs2::humanoid
