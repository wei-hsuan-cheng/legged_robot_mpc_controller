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

#include "humanoid_common_mpc/target/WalkingVelocityTarget.h"

#include <utility>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WalkingVelocityTarget::WalkingVelocityTarget(const ReferenceConfig& referenceConfig, Generator generator)
    : generator_(std::move(generator)),
      maxDisplacementVelocityX_(referenceConfig.maxDisplacementVelocityX),
      maxDisplacementVelocityY_(referenceConfig.maxDisplacementVelocityY),
      maxRotationVelocity_(referenceConfig.maxRotationVelocity),
      velocityCommandFilter_(5, vector4_t::Zero()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WalkingVelocityTarget::setCommand(const WalkingVelocityCommand& command) {
  std::lock_guard<std::mutex> lock(commandMutex_);
  command_ = command;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WalkingVelocityCommand WalkingVelocityTarget::scaleCommand(WalkingVelocityCommand command) const {
  command.linear_velocity_x *= maxDisplacementVelocityX_;
  command.linear_velocity_y *= maxDisplacementVelocityY_;
  command.angular_velocity_z *= maxRotationVelocity_;
  return command;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WalkingVelocityTarget::Output WalkingVelocityTarget::evaluate(scalar_t initTime, scalar_t finalTime, const vector_t& initState) {
  WalkingVelocityCommand latestCommand;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    latestCommand = command_;
  }

  WalkingVelocityCommand scaledCommand = scaleCommand(latestCommand);
  vector4_t conditionedCommand = velocityCommandFilter_.getFilteredVector(scaledCommand.toVector());

  Output output;
  output.conditionedCommand = conditionedCommand;
  output.targetTrajectories = generator_(conditionedCommand, initTime, finalTime, initState);
  return output;
}

}  // namespace ocs2::humanoid
