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

#include "humanoid_centroidal_mpc/CentroidalMpcInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <humanoid_common_mpc/HumanoidCostConstraintFactory.h>
#include <humanoid_common_mpc/HumanoidPreComputation.h>
#include <humanoid_common_mpc/constraint/EndEffectorKinematicsTwistConstraint.h>
#include <humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>

#include "humanoid_centroidal_mpc/constraint/JointMimicKinematicConstraint.h"
#include "humanoid_centroidal_mpc/constraint/NormalVelocityConstraintCppAd.h"
#include "humanoid_centroidal_mpc/constraint/ZeroVelocityConstraintCppAd.h"
#include "humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h"
#include "humanoid_centroidal_mpc/cost/ICPCost.h"
#include "humanoid_centroidal_mpc/dynamics/CentroidalDynamicsAD.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcInterface::CentroidalMpcInterface(Config config, bool setupOCP)
    : modelSettings_(config.modelParams, config.urdfFile, "centroidal_mpc_", config.verbose),
      ddpSettings_(config.ddpSettings),
      mpcSettings_(config.mpcSettings),
      sqpSettings_(config.sqpSettings),
      rolloutSettings_(config.rolloutSettings),
      config_(std::move(config)),
      verbose_(config_.verbose) {
  // check that urdf file exists
  std::ifstream urdfStream(config_.urdfFile);
  if (urdfStream.good()) {
    std::cerr << "[CentroidalMpcInterface] Loading Pinocchio model from: " << config_.urdfFile << std::endl;
  } else {
    throw std::invalid_argument("[CentroidalMpcInterface] URDF file not found: " + config_.urdfFile);
  }

  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createCustomPinocchioInterface(config_.urdfFile, modelSettings_, false)));

  // CentroidalModelInfo
  if (config_.referenceJointState.size() != pinocchioInterfacePtr_->getModel().nq - 6) {
    throw std::invalid_argument("[CentroidalMpcInterface] referenceJointState size (" +
                                std::to_string(config_.referenceJointState.size()) + ") does not match nq - 6 (" +
                                std::to_string(pinocchioInterfacePtr_->getModel().nq - 6) + ")");
  }
  centroidalModelInfo_ =
      centroidal_model::createCentroidalModelInfo(*pinocchioInterfacePtr_, config_.centroidalModelType, config_.referenceJointState,
                                                  modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

  std::cout << "centroidalModelInfo_.numSixDofContacts: " << centroidalModelInfo_.numSixDofContacts << std::endl;
  for (int i = 0; i < centroidalModelInfo_.numSixDofContacts; i++) {
    std::cout << "frameIndices: " << centroidalModelInfo_.endEffectorFrameIndices[i] << std::endl;
  }

  // Setup Centroidal State Input Mapping
  mpcRobotModelPtr_.reset(new CentroidalMpcRobotModel<scalar_t>(modelSettings_, *pinocchioInterfacePtr_, centroidalModelInfo_));
  mpcRobotModelADPtr_.reset(
      new CentroidalMpcRobotModel<ad_scalar_t>(modelSettings_, (*pinocchioInterfacePtr_).toCppAd(), centroidalModelInfo_.toCppAd()));

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(new SwingTrajectoryPlanner(config_.swingTrajectoryConfig, N_CONTACTS));

  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(
      GaitSchedule::createGaitSchedule(config_.initialModeSchedule, config_.defaultModeSequenceTemplate, modelSettings_, verbose_),
      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(true);

  // initial state
  if (config_.initialState.size() != centroidalModelInfo_.stateDim) {
    throw std::invalid_argument("[CentroidalMpcInterface] initialState size (" + std::to_string(config_.initialState.size()) +
                                ") does not match the MPC state dimension (" + std::to_string(centroidalModelInfo_.stateDim) + ")");
  }
  initialState_ = config_.initialState;

  if (setupOCP) {
    setupOptimalControlProblem();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalMpcInterface::setupOptimalControlProblem() {
  HumanoidCostConstraintFactory factory =
      HumanoidCostConstraintFactory(config_.costConstraintConfig, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                    *mpcRobotModelADPtr_, modelSettings_, verbose_);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  const std::string modelName = "dynamics";
  dynamicsPtr.reset(new CentroidalDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());

  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);

  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };

  addTaskSpaceKinematicsCosts(pinocchioMappingCppAd, velocityUpdateCallback);

  const vector2_t icpWeights = config_.icpCostWeights;
  problemPtr_->costPtr->add(
      "icp_Cost", std::unique_ptr<StateInputCost>(new ICPCost(*referenceManagerPtr_, std::move(icpWeights), *pinocchioInterfacePtr_,
                                                              *mpcRobotModelADPtr_, "icp_Cost", modelSettings_)));

  // Constraints
  problemPtr_->stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());

  // Constraint terms
  const EndEffectorKinematicsWeights& footTrackingCostWeights = config_.taskSpaceFootCostWeights;

  // check for mimic joints
  const bool hasMimicJoints = !config_.mimicJoints.empty();

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings_.contactNames[i];

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getStanceFootConstraint(*eeKinematicsPtr, i));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", getNormalVelocityConstraint(*eeKinematicsPtr, i));
    if (hasMimicJoints) {
      problemPtr_->equalityConstraintPtr->add(footName + "_kneeJointMimic", getJointMimicConstraint(i));
    }

    std::string footTrackingCostName = footName + "_TaskSpaceKinematicsCost";

    problemPtr_->costPtr->add(footTrackingCostName, std::unique_ptr<StateInputCost>(new CentroidalMpcEndEffectorFootCost(
                                                        *referenceManagerPtr_, footTrackingCostWeights, *pinocchioInterfacePtr_,
                                                        *mpcRobotModelADPtr_, i, footTrackingCostName, modelSettings_)));
    problemPtr_->costPtr->add(footName + "_ExternalTorqueQuadraticCost", factory.getExternalTorqueQuadraticCost(i));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(
      new HumanoidPreComputation(*pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(
      new CentroidalWeightCompInitializer(centroidalModelInfo_, *referenceManagerPtr_, *mpcRobotModelPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getStanceFootConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain, scalar_t orientationErrorGain) {
    EndEffectorKinematicsTwistConstraint::Config config;
    config.b.setZero(6);
    config.Ax.setZero(6, 6);
    config.Av.setIdentity(6, 6);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax(2, 2) = positionErrorGain;
    }
    if (!numerics::almost_eq(orientationErrorGain, 0.0)) {
      config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * orientationErrorGain;
    }

    return config;
  };

  return std::unique_ptr<StateInputConstraint>(
      new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                      eeZeroVelConConfig(modelSettings_.footConstraintConfig.positionErrorGain_z,
                                                         modelSettings_.footConstraintConfig.orientationErrorGain)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getNormalVelocityConstraint(
    const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getJointMimicConstraint(size_t mimicIndex) {
  if (mimicIndex >= config_.mimicJoints.size()) {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }
  const MimicJointConfig& mimic = config_.mimicJoints[mimicIndex];

  JointMimicKinematicConstraint::Config config(*mpcRobotModelPtr_, mimic.parentJointName, mimic.childJointName, mimic.multiplier,
                                               mimic.positionGain);

  return std::unique_ptr<StateInputConstraint>(new JointMimicKinematicConstraint(*mpcRobotModelPtr_, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalMpcInterface::addTaskSpaceKinematicsCosts(
    const CentroidalModelPinocchioMappingCppAd& pinocchioMappingCppAd,
    const PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback) {
  for (const TaskSpaceCostConfig& taskSpaceCost : config_.taskSpaceCosts) {
    const std::string& costName = taskSpaceCost.costName;
    const std::string& linkName = taskSpaceCost.linkName;

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {linkName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, linkName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    std::unique_ptr<StateInputCost> cost = std::make_unique<EndEffectorKinematicsQuadraticCost>(
        taskSpaceCost.weights, *pinocchioInterfacePtr_, *eeKinematicsPtr, *mpcRobotModelADPtr_, linkName, modelSettings_);

    problemPtr_->costPtr->add(costName + "_TaskSpaceKinematicsCost", std::move(cost));

    std::cout << "Initialized Task Space Kinematics Cost for link: " << linkName << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getCostNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->costPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getTerminalCostNames() const {
  std::vector<std::string> terminalCostNames;
  for (const auto& [costName, index] : problemPtr_->finalCostPtr->getTermNameMap()) {
    terminalCostNames.emplace_back(costName);
  }
  return terminalCostNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getStateSoftConstraintNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->stateSoftConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getSoftConstraintNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->softConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getEqualityConstraintNames() const {
  std::vector<std::string> costNames;
  for (const auto& [costName, index] : problemPtr_->equalityConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

}  // namespace ocs2::humanoid
