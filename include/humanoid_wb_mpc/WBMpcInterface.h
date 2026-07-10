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

#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"
#include "humanoid_common_mpc/initialization/WeightCompInitializer.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"
#include "humanoid_wb_mpc/cost/EndEffectorDynamicsCostHelpers.h"
#include "humanoid_wb_mpc/common/WBAccelMpcRobotModel.h"
#include "humanoid_wb_mpc/end_effector/EndEffectorDynamics.h"

namespace ocs2::humanoid {

class WBMpcInterface final : public RobotInterface {
 public:
  /** Mimic joint constraint configuration (one entry per contact / leg). */
  struct MimicJointConfig {
    std::string parentJointName;
    std::string childJointName;
    scalar_t multiplier{1.0};   // q_child = multiplier * q_parent
    scalar_t positionGain{0.0};
    scalar_t velocityGain{0.0};
  };

  /** Full whole-body MPC configuration, filled from ROS 2 parameters (generate_parameter_library). */
  struct Config {
    std::string urdfFile;
    bool verbose = false;

    ModelSettings::Params modelParams;

    ddp::Settings ddpSettings;
    mpc::Settings mpcSettings;
    sqp::Settings sqpSettings;
    rollout::Settings rolloutSettings;

    ModeSchedule initialModeSchedule{{0.5}, {ModeNumber::STANCE, ModeNumber::STANCE}};
    ModeSequenceTemplate defaultModeSequenceTemplate{{0.0, 0.5}, {ModeNumber::STANCE}};

    SwingTrajectoryPlanner::Config swingTrajectoryConfig;

    vector_t initialState;

    HumanoidCostConstraintFactory::Config costConstraintConfig;
    EndEffectorDynamicsWeights taskSpaceFootCostWeights;
    vector_t jointTorqueWeights;  // optional, empty = joint torque cost disabled

    std::vector<MimicJointConfig> mimicJoints;  // optional, empty = mimic constraints disabled
  };

  /**
   * Constructor
   *
   * @throw Invalid argument error if the urdf file does not exist.
   *
   * @param [in] config: The full whole-body MPC configuration.
   */
  explicit WBMpcInterface(Config config, bool setupOCP = true);

  ~WBMpcInterface() override = default;

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() { return sqpSettings_; }

  const std::string& getURDFFile() const { return config_.urdfFile; }

  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const WeightCompInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const WBAccelMpcRobotModel<scalar_t>& getMpcRobotModel() const { return *mpcRobotModelPtr_; }
  const WBAccelMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() const { return *mpcRobotModelADPtr_; }

 private:
  void setupOptimalControlProblem();

  std::unique_ptr<StateInputConstraint> getStanceFootConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics, size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                    size_t contactPointIndex);
  std::unique_ptr<StateInputCost> getJointTorqueCost();
  std::unique_ptr<StateInputConstraint> getJointMimicConstraint(size_t mimicIndex);

  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  std::unique_ptr<WBAccelMpcRobotModel<scalar_t>> mpcRobotModelPtr_;
  std::unique_ptr<WBAccelMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<WeightCompInitializer> initializerPtr_;

  vector_t initialState_;

  Config config_;
  bool verbose_;
};

}  // namespace ocs2::humanoid
