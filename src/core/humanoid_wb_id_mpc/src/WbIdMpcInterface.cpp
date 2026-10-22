#include <fstream>
#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/WbIdMpcInterface.h"

#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h>
#include <humanoid_common_mpc/HumanoidCostConstraintFactory.h>
#include <humanoid_common_mpc/HumanoidPreComputation.h>
#include <humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>

#include "humanoid_wb_id_mpc/common/WbIdPinocchioMapping.h"
#include "humanoid_wb_id_mpc/constraint/WbIdJointTorqueLimitConstraint.h"
#include "humanoid_wb_id_mpc/constraint/WbIdJointVelocityLimitConstraint.h"
#include "humanoid_wb_id_mpc/constraint/WbIdRneaConstraintCppAd.h"
#include "humanoid_wb_id_mpc/dynamics/WbIdDynamics.h"

namespace ocs2::humanoid {

WbIdMpcInterface::WbIdMpcInterface(Config config, bool setupOCP)
    // The "wb_id_mpc_" prefix keeps the generated CppAD libraries in their own folder:
    // they are cached by model NAME only, and several terms here share their name with
    // a centroidal one built for different state/input dimensions.
    : modelSettings_(config.modelParams, config.urdfFile, "wb_id_mpc_", config.verbose),
      ddpSettings_(config.ddpSettings),
      mpcSettings_(config.mpcSettings),
      sqpSettings_(config.sqpSettings),
      rolloutSettings_(config.rolloutSettings),
      config_(std::move(config)),
      verbose_(config_.verbose) {
  std::ifstream urdfStream(config_.urdfFile);
  if (!urdfStream.good()) {
    throw std::invalid_argument("[WbIdMpcInterface] URDF file not found: " + config_.urdfFile);
  }
  std::cerr << "[WbIdMpcInterface] Loading Pinocchio model from: " << config_.urdfFile << std::endl;

  pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(createCustomPinocchioInterface(config_.urdfFile, modelSettings_, false));
  const auto& model = pinocchioInterfacePtr_->getModel();
  if (model.nq != model.nv || static_cast<size_t>(model.nv) != 6 + modelSettings_.mpc_joint_dim) {
    throw std::invalid_argument("[WbIdMpcInterface] expected nq == nv == 6 + " + std::to_string(modelSettings_.mpc_joint_dim) +
                                " (Translation + SphericalZYX base), got nq=" + std::to_string(model.nq) +
                                " nv=" + std::to_string(model.nv));
  }
  for (const auto& contactName : modelSettings_.contactNames) {
    if (model.getFrameId(contactName) >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
      throw std::invalid_argument("[WbIdMpcInterface] contact frame '" + contactName + "' was not found in the Pinocchio model");
    }
  }

  mpcRobotModelPtr_ = std::make_unique<WbIdMpcRobotModel<scalar_t>>(modelSettings_);
  mpcRobotModelADPtr_ = std::make_unique<WbIdMpcRobotModel<ad_scalar_t>>(modelSettings_);

  // Same gait, swing planner and reference manager as the centroidal MPC.
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(new SwingTrajectoryPlanner(config_.swingTrajectoryConfig, N_CONTACTS));
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(
      GaitSchedule::createGaitSchedule(config_.initialModeSchedule, config_.defaultModeSequenceTemplate, modelSettings_, verbose_),
      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingSettings(config_.armSwing);
  referenceManagerPtr_->setCaptureFootPlacementSettings(config_.captureFootPlacement);
  referenceManagerPtr_->setUseTerrainHeightEstimate(config_.useTerrainHeightEstimate, config_.maxTerrainHeightStep);

  if (config_.initialState.size() != static_cast<Eigen::Index>(mpcRobotModelPtr_->getStateDim())) {
    throw std::invalid_argument("[WbIdMpcInterface] initialState size (" + std::to_string(config_.initialState.size()) +
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

void WbIdMpcInterface::setupOptimalControlProblem() {
  HumanoidCostConstraintFactory factory(config_.costConstraintConfig, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                        *mpcRobotModelADPtr_, modelSettings_, verbose_);

  problemPtr_ = std::make_unique<OptimalControlProblem>();

  // Dynamics: a double integrator; the rigid-body dynamics are the RNEA constraint below.
  problemPtr_->dynamicsPtr = std::make_unique<WbIdDynamics>(*mpcRobotModelPtr_);

  // Costs - the centroidal MPC's set, in the whole-body dimensions.
  problemPtr_->costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  problemPtr_->stateCostPtr->add("baseMotionTrackingCost", factory.getBaseMotionTrackingCost());
  problemPtr_->stateCostPtr->add("jointTrackingCost", factory.getJointTrackingCost());
  problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());
  problemPtr_->finalCostPtr->add("baseMotionTrackingTerminalCost", factory.getBaseMotionTrackingTerminalCost());
  problemPtr_->finalCostPtr->add("jointTrackingTerminalCost", factory.getJointTrackingTerminalCost());

  const WbIdPinocchioMappingCppAd pinocchioMappingCppAd(mpcRobotModelPtr_->getGenCoordinatesDim(), mpcRobotModelPtr_->getInputDim());
  for (const TaskSpaceCostConfig& taskSpaceCost : config_.taskSpaceCosts) {
    PinocchioEndEffectorKinematicsCppAd eeKinematics(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {taskSpaceCost.linkName},
                                                     mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getInputDim(),
                                                     taskSpaceCost.linkName, modelSettings_.modelFolderCppAd,
                                                     modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd);
    problemPtr_->costPtr->add(taskSpaceCost.costName + "_TaskSpaceKinematicsCost",
                              std::make_unique<EndEffectorKinematicsQuadraticCost>(taskSpaceCost.weights, *pinocchioInterfacePtr_,
                                                                                   eeKinematics, *mpcRobotModelADPtr_,
                                                                                   taskSpaceCost.linkName, modelSettings_));
  }

  // State soft constraints.
  problemPtr_->stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());
  if (config_.jointVelocityLimitScale > 0.0) {
    // The URDF velocity limits of the MPC joints, in pinocchio's joint order (= the MPC's).
    const auto nj = static_cast<Eigen::Index>(mpcRobotModelPtr_->getJointDim());
    const vector_t velocityLimits = config_.jointVelocityLimitScale * pinocchioInterfacePtr_->getModel().velocityLimit.tail(nj);
    problemPtr_->stateSoftConstraintPtr->add(
        "jointVelocityLimits",
        std::make_unique<StateSoftConstraint>(std::make_unique<WbIdJointVelocityLimitConstraint>(*mpcRobotModelPtr_, velocityLimits),
                                              std::make_unique<RelaxedBarrierPenalty>(config_.jointVelocityLimitBarrierConfig)));
  }

  // The inverse dynamics: base rows everywhere, joint rows and torque limits in the window.
  torqueWindowPtr_ = std::make_shared<WbIdTorqueWindow>(config_.torqueWindowDuration);
  torqueWindowModulePtr_ = std::make_shared<WbIdTorqueWindowSynchronizedModule>(torqueWindowPtr_);
  problemPtr_->equalityConstraintPtr->add(
      "rneaBaseDynamics", std::make_unique<WbIdRneaConstraintCppAd>(WbIdRneaConstraintCppAd::Rows::Base, *pinocchioInterfacePtr_,
                                                                    *mpcRobotModelADPtr_, modelSettings_));
  problemPtr_->equalityConstraintPtr->add(
      "rneaJointTorques", std::make_unique<WbIdRneaConstraintCppAd>(WbIdRneaConstraintCppAd::Rows::Joint, *pinocchioInterfacePtr_,
                                                                    *mpcRobotModelADPtr_, modelSettings_, torqueWindowPtr_));
  if (config_.torqueLimits.size() > 0) {
    problemPtr_->softConstraintPtr->add(
        "jointTorqueLimits",
        std::make_unique<StateInputSoftConstraint>(
            std::make_unique<WbIdJointTorqueLimitConstraint>(*mpcRobotModelPtr_, config_.torqueLimits, torqueWindowPtr_),
            std::make_unique<RelaxedBarrierPenalty>(config_.torqueLimitBarrierConfig)));
  }

  // Contacts.
  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings_.contactNames[i];

    problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    problemPtr_->equalityConstraintPtr->add(
        footName + "_stanceAcceleration",
        std::make_unique<WbIdFootAccelerationConstraintCppAd>(WbIdFootAccelerationConstraintCppAd::Type::Stance, *referenceManagerPtr_,
                                                              *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, i,
                                                              config_.footAccelerationGains, modelSettings_));
    problemPtr_->equalityConstraintPtr->add(
        footName + "_swingVerticalAcceleration",
        std::make_unique<WbIdFootAccelerationConstraintCppAd>(WbIdFootAccelerationConstraintCppAd::Type::SwingVertical,
                                                              *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, i,
                                                              config_.footAccelerationGains, modelSettings_));

    // Swing foot orientation/velocity tracking and the capture-point foothold: the
    // centroidal MPC's foot cost, which is model agnostic (q, v through the robot model).
    const std::string footTrackingCostName = footName + "_TaskSpaceKinematicsCost";
    problemPtr_->costPtr->add(footTrackingCostName, std::make_unique<CentroidalMpcEndEffectorFootCost>(
                                                        *referenceManagerPtr_, config_.taskSpaceFootCostWeights, *pinocchioInterfacePtr_,
                                                        *mpcRobotModelADPtr_, i, footTrackingCostName, modelSettings_));
    problemPtr_->costPtr->add(footName + "_ExternalTorqueQuadraticCost", factory.getExternalTorqueQuadraticCost(i));
  }

  problemPtr_->preComputationPtr =
      std::make_unique<HumanoidPreComputation>(*pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_);

  rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  initializerPtr_ = std::make_unique<WbIdInitializer>(*pinocchioInterfacePtr_, *referenceManagerPtr_, *mpcRobotModelPtr_);
}

}  // namespace ocs2::humanoid
