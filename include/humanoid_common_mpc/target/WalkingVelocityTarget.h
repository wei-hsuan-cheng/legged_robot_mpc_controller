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
#include <mutex>

#include <ocs2_core/reference/TargetTrajectories.h>

#include "humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h"
#include "humanoid_common_mpc/command/WalkingVelocityCommand.h"
#include "humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h"

namespace ocs2::humanoid {

/**
 * Single owner of the walking-velocity command -> TargetTrajectories path.
 *
 * It consolidates the three command-conditioning stages that used to be spread
 * across the ROS callback and the motion manager: scale the bounded command to
 * physical units, low-pass filter it, and generate the reference trajectory
 * through the injected per-model calculator (which also applies the heading
 * hold). evaluate() returns both the TargetTrajectories for the reference
 * manager and the conditioned command vector consumed by the gait selector, so
 * scaling and filtering happen exactly once per solver step.
 */
class WalkingVelocityTarget {
 public:
  /// Transforms a conditioned velocity command into a reference trajectory.
  using Generator =
      std::function<TargetTrajectories(const vector4_t& velocityTarget, scalar_t initTime, scalar_t finalTime, const vector_t& initState)>;

  struct Output {
    TargetTrajectories targetTrajectories;
    vector4_t conditionedCommand;  //!< scaled + filtered [v_x, v_y, pelvis_height, v_yaw]
  };

  WalkingVelocityTarget(const ReferenceConfig& referenceConfig, Generator generator);

  /// Store the latest bounded command. Thread-safe (called from the ROS callback thread).
  void setCommand(const WalkingVelocityCommand& command);

  /// Scale + filter the latest command and build the reference trajectory.
  /// Must only be called from the solver thread (owns the filter state).
  Output evaluate(scalar_t initTime, scalar_t finalTime, const vector_t& initState);

 private:
  WalkingVelocityCommand scaleCommand(WalkingVelocityCommand command) const;

  Generator generator_;
  scalar_t maxDisplacementVelocityX_;
  scalar_t maxDisplacementVelocityY_;
  scalar_t maxRotationVelocity_;

  BreakFrequencyAlphaFilter velocityCommandFilter_;

  mutable std::mutex commandMutex_;
  WalkingVelocityCommand command_;
};

}  // namespace ocs2::humanoid
