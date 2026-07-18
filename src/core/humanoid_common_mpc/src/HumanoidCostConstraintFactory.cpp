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

#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"

#include <ocs2_core/penalties/Penalties.h>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <humanoid_common_mpc/constraint/FrictionForceConeConstraint.h>
#include <humanoid_common_mpc/constraint/ZeroWrenchConstraint.h>
#include <humanoid_common_mpc/contact/ContactRectangle.h>
#include <humanoid_common_mpc/cost/StateInputQuadraticCost.h>
#include <humanoid_common_mpc/cost/BaseMotionTrackingCost.h>
#include <humanoid_common_mpc/cost/JointTrackingCost.h>
#include <humanoid_common_mpc/pinocchio_model/pinocchioUtils.h>
#include "humanoid_common_mpc/constraint/ContactMomentXYConstraintCppAd.h"
#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"
#include "humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h"
#include "humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h"

namespace ocs2::humanoid {

HumanoidCostConstraintFactory::HumanoidCostConstraintFactory(Config config,
                                                             const SwitchedModelReferenceManager& referenceManager,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                             const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                                             const ModelSettings& modelSettings,
                                                             bool verbose)
    : config_(std::move(config)),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfacePtr_(&pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel),
      mpcRobotModelADPtr_(&mpcRobotModelAD),
      modelSettings_(modelSettings),
      verbose_(verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getStateInputQuadraticCost() const {
  if (config_.Q.rows() != mpcRobotModelADPtr_->getStateDim() || config_.R.rows() != mpcRobotModelADPtr_->getInputDim()) {
    throw std::invalid_argument("[HumanoidCostConstraintFactory] Q/R dimensions do not match the MPC state/input dimensions");
  }
  matrix_t Q = config_.Q;
  matrix_t R = config_.R;

  if (verbose_) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(
      new StateInputQuadraticCost(std::move(Q), std::move(R), *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getBaseMotionTrackingCost() const
{
  return std::make_unique<BaseMotionTrackingCost>(
    config_.baseMotionQ, *referenceManagerPtr_, *mpcRobotModelPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getBaseMotionTrackingTerminalCost() const
{
  return std::make_unique<BaseMotionTrackingCost>(
    config_.baseMotionQFinal * config_.terminalCostScaling,
    *referenceManagerPtr_, *mpcRobotModelPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getJointTrackingCost() const
{
  // Running term: tracks the arm-swing reference (matches the running StateInputQuadraticCost).
  return std::make_unique<JointTrackingCost>(
    config_.armJointQ, config_.armJointIndices, *referenceManagerPtr_, *mpcRobotModelPtr_,
    /*apply_arm_swing_reference=*/true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getJointTrackingTerminalCost() const
{
  // Terminal term: tracks the raw posture reference (matches the terminal QuadraticStateCost).
  return std::make_unique<JointTrackingCost>(
    config_.armJointQFinal * config_.terminalCostScaling, config_.armJointIndices,
    *referenceManagerPtr_, *mpcRobotModelPtr_, /*apply_arm_swing_reference=*/false);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getFootCollisionConstraint() const {
  FootCollisionConstraint::Config collisionConfig = config_.footCollisionConfig;

  std::unique_ptr<PieceWisePolynomialBarrierPenalty> penalty(new PieceWisePolynomialBarrierPenalty(config_.footCollisionBarrierConfig));

  std::unique_ptr<FootCollisionConstraint> footCollisionConstraintPtr(
      new FootCollisionConstraint(*referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, std::move(collisionConfig),
                                  "FootCollisionConstraint", modelSettings_));

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footCollisionConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getJointLimitsConstraint() const {
  const PieceWisePolynomialBarrierPenalty::Config& barrierPenaltyConfig = config_.jointLimitsBarrierConfig;

  std::cout << "Initialized joint limits constraint with zero crossing cost " << barrierPenaltyConfig.getZeroCrossingValue() << "."
            << std::endl;

  std::pair<vector_t, vector_t> jointLimits = readPinocchioJointLimits(*pinocchioInterfacePtr_, mpcRobotModelPtr_->modelSettings);

  return std::unique_ptr<StateCost>(new JointLimitsSoftConstraint(jointLimits, barrierPenaltyConfig, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getContactMomentXYConstraint(size_t contactPointIndex,
                                                                                            const std::string& name) const {
  std::unique_ptr<ContactMomentXYConstraintCppAd> contactMomentXYConstraintPtr(new ContactMomentXYConstraintCppAd(
      *referenceManagerPtr_, ContactRectangle::fromModelSettings(mpcRobotModelPtr_->modelSettings, contactPointIndex), contactPointIndex,
      *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, name, modelSettings_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(config_.contactMomentBarrierConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(contactMomentXYConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> HumanoidCostConstraintFactory::getZeroWrenchConstraint(size_t contactPointIndex) const {
  return std::unique_ptr<StateInputConstraint>(new ZeroWrenchConstraint(*referenceManagerPtr_, contactPointIndex, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getFrictionForceConeConstraint(size_t contactPointIndex) const {
  FrictionForceConeConstraint::Config frictionConeConConfig(config_.frictionCoefficient);
  std::unique_ptr<FrictionForceConeConstraint> frictionForceConeConstraintPtr(
      new FrictionForceConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, *mpcRobotModelPtr_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(config_.frictionBarrierConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionForceConeConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getTerminalCost() const {
  if (config_.QFinal.rows() != mpcRobotModelPtr_->getStateDim()) {
    throw std::invalid_argument("[HumanoidCostConstraintFactory] Q_final dimension does not match the MPC state dimension");
  }
  matrix_t Qf = config_.QFinal * config_.terminalCostScaling;
  if (verbose_) std::cerr << "Q_final:\n" << Qf << std::endl;
  return std::unique_ptr<StateCost>(new QuadraticStateCost(Qf));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getExternalTorqueQuadraticCost(size_t contactPointIndex) const {
  const ExternalTorqueQuadraticCostAD::Config& config =
      (contactPointIndex == 0) ? config_.leftLegTorqueCostConfig : config_.rightLegTorqueCostConfig;
  return std::make_unique<ExternalTorqueQuadraticCostAD>(contactPointIndex, config, *referenceManagerPtr_, *pinocchioInterfacePtr_,
                                                         *mpcRobotModelADPtr_, modelSettings_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid
