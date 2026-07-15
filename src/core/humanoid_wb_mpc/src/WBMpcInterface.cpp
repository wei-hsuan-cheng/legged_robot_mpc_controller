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

#include <fstream>
#include <iostream>
#include <string>

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_mpc/WBMpcInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"
#include "humanoid_common_mpc/initialization/WeightCompInitializer.h"

#include "humanoid_wb_mpc/WBMpcPreComputation.h"
#include "humanoid_wb_mpc/constraint/JointMimicDynamicsConstraint.h"
#include "humanoid_wb_mpc/constraint/SwingLegVerticalConstraintCppAd.h"
#include "humanoid_wb_mpc/constraint/ZeroAccelerationConstraintCppAd.h"
#include "humanoid_wb_mpc/cost/EndEffectorDynamicsFootCost.h"
#include "humanoid_wb_mpc/cost/JointTorqueCostCppAd.h"
#include "humanoid_wb_mpc/dynamics/WBAccelDynamicsAD.h"
#include "humanoid_wb_mpc/end_effector/PinocchioEndEffectorDynamicsCppAd.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WBMpcInterface::WBMpcInterface(Config config, bool setupOCP)
    : modelSettings_(config.modelParams, config.urdfFile, "wb_mpc_", config.verbose),
      ddpSettings_(config.ddpSettings),
      mpcSettings_(config.mpcSettings),
      sqpSettings_(config.sqpSettings),
      rolloutSettings_(config.rolloutSettings),
      config_(std::move(config)),
      verbose_(config_.verbose) {
  // check that urdf file exists
  std::ifstream urdfStream(config_.urdfFile);
  if (urdfStream.good()) {
    std::cerr << "[WBMpcInterface] Loading Pinocchio model from: " << config_.urdfFile << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] URDF file not found: " + config_.urdfFile);
  }

  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createCustomPinocchioInterface(config_.urdfFile, modelSettings_)));

  // Setup WB State Input Mapping
  mpcRobotModelPtr_.reset(new WBAccelMpcRobotModel<scalar_t>(modelSettings_));
  mpcRobotModelADPtr_.reset(new WBAccelMpcRobotModel<ad_scalar_t>(modelSettings_));

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(new SwingTrajectoryPlanner(config_.swingTrajectoryConfig, N_CONTACTS));

  // Mode schedule manager
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(
      GaitSchedule::createGaitSchedule(config_.initialModeSchedule, config_.defaultModeSequenceTemplate, modelSettings_, verbose_),
      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(true);

  // initial state
  if (config_.initialState.size() != mpcRobotModelPtr_->getStateDim()) {
    throw std::invalid_argument("[WBMpcInterface] initialState size (" + std::to_string(config_.initialState.size()) +
                                ") does not match the MPC state dimension (" + std::to_string(mpcRobotModelPtr_->getStateDim()) + ")");
  }
  initialState_ = config_.initialState;

  if (setupOCP) {
    setupOptimalControlProblem();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcInterface::setupOptimalControlProblem() {
  HumanoidCostConstraintFactory factory =
      HumanoidCostConstraintFactory(config_.costConstraintConfig, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                    *mpcRobotModelADPtr_, modelSettings_, verbose_);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  const std::string modelName = "dynamics";
  dynamicsPtr.reset(new WBAccelDynamicsAD(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, modelName, modelSettings_));

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  problemPtr_->stateCostPtr->add("baseMotionTrackingCost", factory.getBaseMotionTrackingCost());
  // problemPtr_->costPtr->add("jointTorqueCost", getJointTorqueCost());
  problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());
  problemPtr_->finalCostPtr->add(
      "baseMotionTrackingTerminalCost", factory.getBaseMotionTrackingTerminalCost());

  // Constraints
  problemPtr_->stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());
  // Constraint terms

  const EndEffectorDynamicsWeights& footTrackingCostWeights = config_.taskSpaceFootCostWeights;

  // check for mimic joints
  const bool hasMimicJoints = !config_.mimicJoints.empty();

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings_.contactNames[i];

    std::unique_ptr<EndEffectorDynamics<scalar_t>> eeDynamicsPtr;
    eeDynamicsPtr.reset(new PinocchioEndEffectorDynamicsCppAd(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, {footName}, footName,
                                                              modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd,
                                                              modelSettings_.verboseCppAd));

    problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getStanceFootConstraint(*eeDynamicsPtr, i));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", getNormalVelocityConstraint(*eeDynamicsPtr, i));

    if (hasMimicJoints) {
      problemPtr_->equalityConstraintPtr->add(footName + "_kneeJointMimic", getJointMimicConstraint(i));
    }

    std::string footTrackingCostName = footName + "_TaskSpaceTrackingCost";

    problemPtr_->costPtr->add(footTrackingCostName, std::unique_ptr<StateInputCost>(new EndEffectorDynamicsFootCost(
                                                        *referenceManagerPtr_, footTrackingCostWeights, *pinocchioInterfacePtr_,
                                                        *eeDynamicsPtr, *mpcRobotModelADPtr_, i, footTrackingCostName, modelSettings_)));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(
      new WBMpcPreComputation(*pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  initializerPtr_.reset(new WeightCompInitializer(*pinocchioInterfacePtr_, *referenceManagerPtr_, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> WBMpcInterface::getStanceFootConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                              size_t contactPointIndex) {
  const ModelSettings::FootConstraintConfig& footCfg = modelSettings_.footConstraintConfig;

  EndEffectorDynamicsAccelerationsConstraint::Config config;
  config.b.setZero(6);
  config.Ax.setZero(6, 6);
  config.Av.setIdentity(6, 6);
  config.Aa.setIdentity(6, 6);
  if (!numerics::almost_eq(footCfg.positionErrorGain_z, 0.0)) {
    config.Ax(2, 2) = footCfg.positionErrorGain_z;
  }
  if (!numerics::almost_eq(footCfg.orientationErrorGain, 0.0)) {
    config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.orientationErrorGain;
  }
  config.Av.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * footCfg.linearVelocityErrorGain_xy;
  config.Av(2, 2) = footCfg.linearVelocityErrorGain_z;
  config.Av.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.angularVelocityErrorGain;
  config.Aa.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * footCfg.linearAccelerationErrorGain_xy;
  config.Aa(2, 2) = footCfg.linearAccelerationErrorGain_z;
  config.Aa.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.angularAccelerationErrorGain;

  return std::unique_ptr<StateInputConstraint>(
      new ZeroAccelerationConstraintCppAd(*referenceManagerPtr_, eeDynamics, contactPointIndex, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> WBMpcInterface::getJointMimicConstraint(size_t mimicIndex) {
  if (mimicIndex >= config_.mimicJoints.size()) {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }
  const MimicJointConfig& mimic = config_.mimicJoints[mimicIndex];

  JointMimicDynamicsConstraint::Config config(*mpcRobotModelPtr_, mimic.parentJointName, mimic.childJointName, mimic.multiplier,
                                              mimic.positionGain, mimic.velocityGain);

  return std::unique_ptr<StateInputConstraint>(new JointMimicDynamicsConstraint(*mpcRobotModelPtr_, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> WBMpcInterface::getNormalVelocityConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                                  size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new SwingLegVerticalConstraintCppAd(*referenceManagerPtr_, eeDynamics, contactPointIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> WBMpcInterface::getJointTorqueCost() {
  if (config_.jointTorqueWeights.size() != mpcRobotModelPtr_->getJointDim()) {
    throw std::invalid_argument("[WBMpcInterface] jointTorqueWeights size does not match the MPC joint dimension");
  }
  return std::unique_ptr<StateInputCost>(new JointTorqueCostCppAd(config_.jointTorqueWeights, *pinocchioInterfacePtr_,
                                                                  *mpcRobotModelADPtr_, "jointTorqueCost", modelSettings_));
}

}  // namespace ocs2::humanoid
