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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>

#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"
#include "humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to constrain the contact moment in the x-y plane.
 */

class HumanoidCostConstraintFactory {
 public:
  /** Cost and constraint configuration, filled from ROS 2 parameters (generate_parameter_library). */
  struct Config {
    matrix_t Q;       // (stateDim x stateDim) state cost matrix
    matrix_t R;       // (inputDim x inputDim) input cost matrix
    matrix_t QFinal;  // (stateDim x stateDim) terminal state cost matrix (unscaled)
    matrix_t baseMotionQ;       // (12 x 12) base pose/motion tracking cost matrix
    matrix_t baseMotionQFinal;  // (12 x 12) terminal base pose/motion tracking cost matrix
    // Arm joint-tracking term (subset of the joint block, addressed by joint-block index).
    std::vector<size_t> armJointIndices;
    matrix_t armJointQ;       // (nArm x nArm) running arm joint-position tracking cost matrix
    matrix_t armJointQFinal;  // (nArm x nArm) terminal arm joint-position tracking cost matrix
    scalar_t terminalCostScaling{1.0};

    FootCollisionConstraint::Config footCollisionConfig;
    PieceWisePolynomialBarrierPenalty::Config footCollisionBarrierConfig;

    PieceWisePolynomialBarrierPenalty::Config jointLimitsBarrierConfig;

    RelaxedBarrierPenalty::Config contactMomentBarrierConfig;

    scalar_t frictionCoefficient{1.0};
    RelaxedBarrierPenalty::Config frictionBarrierConfig;

    ExternalTorqueQuadraticCostAD::Config leftLegTorqueCostConfig;
    ExternalTorqueQuadraticCostAD::Config rightLegTorqueCostConfig;
  };

  HumanoidCostConstraintFactory(Config config,
                                const SwitchedModelReferenceManager& referenceManager,
                                const PinocchioInterface& pinocchioInterface,
                                const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                const ModelSettings& modelSettings,
                                bool verbose = false);

  ~HumanoidCostConstraintFactory() = default;
  HumanoidCostConstraintFactory(const HumanoidCostConstraintFactory& other) = delete;

  std::unique_ptr<StateInputCost> getStateInputQuadraticCost() const;

  std::unique_ptr<StateCost> getBaseMotionTrackingCost() const;

  std::unique_ptr<StateCost> getBaseMotionTrackingTerminalCost() const;

  std::unique_ptr<StateCost> getJointTrackingCost() const;

  std::unique_ptr<StateCost> getJointTrackingTerminalCost() const;

  std::unique_ptr<StateCost> getTerminalCost() const;

  std::unique_ptr<StateCost> getFootCollisionConstraint() const;

  std::unique_ptr<StateCost> getJointLimitsConstraint() const;

  std::unique_ptr<StateInputCost> getContactMomentXYConstraint(size_t contactPointIndex, const std::string& name) const;

  std::unique_ptr<StateInputConstraint> getZeroWrenchConstraint(size_t contactPointIndex) const;

  std::unique_ptr<StateInputCost> getFrictionForceConeConstraint(size_t contactPointIndex) const;

  std::unique_ptr<StateInputCost> getExternalTorqueQuadraticCost(size_t contactPointIndex) const;

 private:
  Config config_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const PinocchioInterface* pinocchioInterfacePtr_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  const MpcRobotModelBase<ad_scalar_t>* mpcRobotModelADPtr_;
  const ModelSettings& modelSettings_;
  const bool verbose_;
};

}  // namespace ocs2::humanoid
