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

#include "humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h"

#include <array>
#include <boost/proto/proto_fwd.hpp>
#include <cmath>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2::humanoid {

CentroidalMpcTargetTrajectoriesCalculator::CentroidalMpcTargetTrajectoriesCalculator(const ReferenceConfig& referenceConfig,
                                                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                                     PinocchioInterface pinocchioInterface,
                                                                                     const CentroidalModelInfo& info,
                                                                                     scalar_t mpcHorizon,
                                                                                     bool useInertiaWeightedAngularMomentum)
    : TargetTrajectoriesCalculatorBase(referenceConfig, mpcRobotModel, mpcHorizon),
      pinocchioInterface_(std::move(pinocchioInterface)),
      mass_(pinocchio::computeTotalMass(pinocchioInterface_.getModel())),
      useInertiaWeightedAngularMomentum_(useInertiaWeightedAngularMomentum) {
  static_cast<void>(info);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector3_t CentroidalMpcTargetTrajectoriesCalculator::normalizedAngularMomentumForYawRate(const vector_t& initState,
                                                                                         scalar_t yawRate) {
  if (!useInertiaWeightedAngularMomentum_) {
    // Historical form, kept only so the correction can be cross-tested against
    // the cost tuning that was fitted around it: omega_z / m, with the x and y
    // angular components pinned to zero.
    return vector3_t(0.0, 0.0, yawRate / mass_);
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(initState);
  pinocchio::ccrba(model, data, q, vector_t::Zero(model.nv));

  // h_ang = I_G * omega, with omega = (0, 0, yawRate): the third COLUMN of I_G
  // scaled by the yaw rate. Normalized by mass to match the centroidal state.
  const matrix3_t Ig = data.Ig.inertia().matrix();
  return (Ig.col(2) * yawRate) / mass_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories(const vector4_t& commadLinePoseTarget,
                                                                                                    scalar_t initTime,
                                                                                                    const vector_t& initState) {
  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);

  const vector_t targetPose = getDeltaBaseTarget(commadLinePoseTarget, currentPoseTarget);

  scalar_t targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPoseTarget);

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << vector_t::Zero(6), currentPoseTarget, targetJointState_;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, targetJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedBasePoseToTargetTrajectories(
    const vector6_t& targetBasePose, scalar_t initTime, const vector_t& initState) {
  const vector6_t currentPose = mpcRobotModelPtr_->getBasePose(initState);
  const scalar_t targetReachingTime = initTime + estimateTimeToBasePoseTarget(targetBasePose - currentPose);

  const scalar_array_t timeTrajectory{initTime, targetReachingTime};
  vector_array_t stateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, targetJointState_;
  stateTrajectory[1] << vector_t::Zero(6), targetBasePose, targetJointState_;

  const vector_array_t inputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));
  return TargetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                                                    scalar_t initTime,
                                                                                                    const vector_t& initState) {
  // Re-anchor every horizon at the current pose, then integrate only the
  // pelvis-frame command. In particular, estimator velocity noise must not
  // move the reference trajectory itself.
  vector6_t currentPoseTarget = getCurrentBasePoseTarget(initState);
  currentPoseTarget(2) = commandedVelocities(2);
  const scalar_t intermediateTargetTime = 0.7 * mpcHorizon_;
  const vector6_t intermediateTargetPose =
      integrateBodyTwistTargetBasePose(currentPoseTarget, commandedVelocities, intermediateTargetTime);
  const vector6_t finalTargetPose =
      integrateBodyTwistTargetBasePose(currentPoseTarget, commandedVelocities, mpcHorizon_);

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, initTime + intermediateTargetTime, initTime + mpcHorizon_};

  // desired state trajectory
  vector_array_t stateTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  const std::array<vector6_t, 3> poses{currentPoseTarget, intermediateTargetPose, finalTargetPose};
  for (size_t i = 0; i < poses.size(); ++i) {
    const vector4_t velocityWorld = transformVelCommandToGlobal(commandedVelocities, poses[i](3));
    // Angular part is I_G(q) * omega / m, not omega / m - see
    // normalizedAngularMomentumForYawRate().
    const vector3_t angularMomentum = normalizedAngularMomentumForYawRate(initState, velocityWorld(3));
    vector6_t targetMomentum;
    targetMomentum << velocityWorld(0), velocityWorld(1), 0.0, angularMomentum;
    stateTrajectory[i] << targetMomentum, poses[i], targetJointState_;
  }

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

}  // namespace ocs2::humanoid
