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

#pragma once

#include <functional>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/Types.h>
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

namespace ocs2::humanoid {

/** Reference / command generation settings, filled from ROS 2 parameters (generate_parameter_library). */
struct ReferenceConfig {
  scalar_t targetDisplacementVelocity{0.5};
  scalar_t targetRotationVelocity{0.5};
  scalar_t maxDisplacementVelocityX{2.0};
  scalar_t maxDisplacementVelocityY{1.2};
  scalar_t maxRotationVelocity{1.0};
  scalar_t defaultBaseHeight{0.79};
  // Commanded base height range [m], above the ground the model assumes; must contain defaultBaseHeight.
  scalar_t minBaseHeight{0.39};
  scalar_t maxBaseHeight{1.19};
  // Physical command slew limits [ax, ay, height rate, yaw acceleration]; zero disables an axis.
  vector4_t commandRateLimits{vector4_t::Zero()};
  scalar_t basePosePositionTolerance{0.02};
  scalar_t basePoseOrientationTolerance{0.03};
  vector_t defaultJointState;
  // Optional joint posture reference as a function of base height: joint states at strictly increasing
  // heights covering [minBaseHeight, maxBaseHeight], linearly interpolated at the target height. Empty: the
  // target joint state at every height.
  scalar_array_t postureHeights;
  vector_array_t postureJointStates;
};

class TargetTrajectoriesCalculatorBase {
 public:
  TargetTrajectoriesCalculatorBase(const ReferenceConfig& referenceConfig,
                                   const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                   scalar_t mpcHorizon);

  TargetTrajectoriesCalculatorBase(const TargetTrajectoriesCalculatorBase& rhs) = delete;

  void setTargetDisplacementVelocity(scalar_t targetDisplacementVelocity) { targetDisplacementVelocity_ = targetDisplacementVelocity; }
  void setTargetRotationVelocity_(scalar_t targetRotationVelocity) { targetRotationVelocity = targetRotationVelocity; }
  void setTargetJointState(const vector_t targetJointState);

  /**
   * Base height clamped to [minBaseHeight, maxBaseHeight] when a posture table is set; unchanged without one,
   * since base pose and terrain targets then carry world-frame heights.
   */
  scalar_t clampBaseHeight(scalar_t height) const;

  /** Joint posture reference at a base height: interpolated from the posture table, else the target joint state. */
  vector_t getPostureForHeight(scalar_t height) const;

  /**
   * Converts command line to TargetTrajectories.
   * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  virtual TargetTrajectories commandedPositionToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                   scalar_t initTime,
                                                                   const vector_t& initState) = 0;

  /** Converts an absolute global-frame [x, y, z, yaw, pitch, roll] target. */
  virtual TargetTrajectories commandedBasePoseToTargetTrajectories(const vector6_t& targetBasePose,
                                                                   scalar_t initTime,
                                                                   const vector_t& initState) = 0;

  /**
   * Converts desired velocities to TargetTrajectories.
   * @param [in] commandedVelocities : [v_x, v_y, pelvis_height, yaw_rate],
   *                                    with the planar twist in the pelvis frame
   * @param [in] observation : the current observation
   */
  virtual TargetTrajectories commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                   scalar_t initTime,
                                                                   const vector_t& initState) = 0;

 protected:
  scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const;

  scalar_t estimateTimeToBasePoseTarget(const vector6_t& desiredBaseDisplacement) const;

  virtual vector6_t getCurrentBasePoseTarget(const vector_t& state) const;

  vector6_t getDeltaBaseTarget(const vector4_t& commadLinePoseTarget, const vector6_t& currentPoseTarget) const;

  vector4_t transformVelCommandToGlobal(const vector4_t& commandedVelLocal,
                                        const scalar_t& currentEulerZ) const;

  /**
   * Integrates a constant pelvis-frame planar twist from the current pose using
   * the SE(2) exponential map. The target depends only on the command and the
   * current pose anchor; measured velocity is deliberately not used.
   */
  vector6_t integrateBodyTwistTargetBasePose(const vector6_t& currentPose,
                                             const vector4_t& commandedVelLocal,
                                             scalar_t deltaT) const;

  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;

  // For pose control mode
  scalar_t targetDisplacementVelocity_;
  scalar_t targetRotationVelocity_;

  // For velocity control mode
  scalar_t maxDisplacementVelocityX_ = 0.6;
  scalar_t maxDisplacementVelocityY_ = 0.3;
  scalar_t maxRotationVelocity_ = 0.6;

  scalar_t defaultBaseHeight_;
  scalar_t minBaseHeight_;
  scalar_t maxBaseHeight_;
  vector_t targetJointState_;
  scalar_array_t postureHeights_;
  vector_array_t postureJointStates_;
  scalar_t mpcHorizon_;
};

}  // namespace ocs2::humanoid
