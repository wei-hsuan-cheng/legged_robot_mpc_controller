#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"
#include "humanoid_wb_id_mpc/constraint/ArmBodyCollisionConstraint.h"
#include "humanoid_wb_id_mpc/cost/WbIdTorsoTiltCost.h"
#include "humanoid_wb_id_mpc/constraint/WbIdFootAccelerationConstraintCppAd.h"
#include "humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h"
#include "humanoid_wb_id_mpc/initialization/WbIdInitializer.h"

namespace ocs2::humanoid {

/**
 * Whole-body inverse-dynamics MPC (Molnar et al., "Whole-Body Inverse Dynamics MPC for
 * Legged Loco-Manipulation", RA-L 2025; Khazoom et al., "Tailoring Solution Accuracy for
 * Fast Whole-Body Model Predictive Control of Legged Robots", RA-L 2024).
 *
 *   x = [q, v]                 (2 nv,         58 on the G1)
 *   u = [W_l, W_r, a, tau_j]   (12 + nv + nj, 64 on the G1)
 *   xdot = [v; a]
 *   s.t. RNEA base rows                       = 0         every node
 *        RNEA joint rows - tau                = 0         first nodes (torque window)
 *        |tau| <= tau_max (soft)                          first nodes (torque window)
 *        stance foot: a_c + Kd v_c + Kp e_c   = 0         i.e. v_c = 0 at the next node
 *        swing foot:  vertical spline tracking, W = 0
 *        friction cone, CoP, joint position and velocity limits, foot collision (soft)
 *
 * Contact is imposed on the foot acceleration rather than on v_c directly (as the ETH
 * formulation does) because OCS2's SQP enforces state-input equalities only: a pure state
 * equality is transcribed but never handed to the QP. With Kd = 1/dt and the exact
 * double-integrator discretization, a_c + v_c/dt = 0 at node k is v_c = 0 at node k + 1.
 *
 * Gait, swing trajectories, capture-point foot placement and the arm posture reference
 * all come from the same SwitchedModelReferenceManager the centroidal MPC uses, so the
 * two controllers walk the same way and differ only in the model they plan with.
 */
class WbIdMpcInterface final : public RobotInterface {
 public:
  struct TaskSpaceCostConfig {
    bool activate = true;
    std::string costName;
    std::string linkName;
    EndEffectorKinematicsWeights weights;
  };

  struct Config {
    std::string urdfFile;
    bool verbose = false;

    ModelSettings::Params modelParams;

    mpc::Settings mpcSettings;
    sqp::Settings sqpSettings;

    ModeSchedule initialModeSchedule{{0.5}, {ModeNumber::STANCE, ModeNumber::STANCE}};
    ModeSequenceTemplate defaultModeSequenceTemplate{{0.0, 0.5}, {ModeNumber::STANCE}};
    SwingTrajectoryPlanner::Config swingTrajectoryConfig;

    vector_t initialState;

    //! Q, R and QFinal in the whole-body dimensions; the rest as in the centroidal MPC.
    HumanoidCostConstraintFactory::Config costConstraintConfig;
    bool activateTaskSpaceFootCost = true;
    EndEffectorKinematicsWeights taskSpaceFootCostWeights;
    bool activateLegTorqueCost = true;
    WbIdTorsoTiltCost::Config torsoTiltCost;

    //! Acceleration-level stance/swing foot constraint gains.
    WbIdFootAccelerationConstraintCppAd::Gains footAccelerationGains;

    bool activateInverseDynamics = true;
    bool activateFootMotionEquality = true;
    bool activateSwingFootZeroWrench = true;
    bool activateFrictionCone = true;
    bool activateContactMomentXY = true;
    bool activateJointPositionLimits = true;
    bool activateJointVelocityLimits = true;
    bool activateJointTorqueLimits = true;
    bool activateLegSelfCollision = true;
    ArmBodyCollisionConfig armSelfCollision;

    //! Time span from the start of the horizon in which the joint-torque rows of the
    //! inverse dynamics and the torque limits apply; < 0 for the whole horizon.
    scalar_t torqueWindowDuration = -1.0;
    //! Optional |tau| override in MPC joint order; empty uses the URDF effort limits.
    //! Shared by the OCP torque-window penalty and runtime feedforward clipping.
    vector_t torqueLimits;
    scalar_t torqueScaling = 1.0;
    RelaxedBarrierPenalty::Config torqueLimitBarrierConfig{0.1, 5.0};

    //! |qd_j| <= velocityScaling * the URDF velocity limit on every node.
    scalar_t velocityScaling = 1.0;
    RelaxedBarrierPenalty::Config jointVelocityLimitBarrierConfig{0.1, 1.0};

    SwitchedModelReferenceManager::CaptureFootPlacementSettings captureFootPlacement;
    SwitchedModelReferenceManager::ArmSwingSettings armSwing;
    bool useTerrainHeightEstimate = false;
    scalar_t maxTerrainHeightStep = 0.01;

    std::vector<TaskSpaceCostConfig> taskSpaceCosts;
  };

  explicit WbIdMpcInterface(Config config, bool setupOCP = true);
  ~WbIdMpcInterface() override = default;

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const sqp::Settings& sqpSettings() const { return sqpSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  const vector_t& getTorqueLimits() const { return config_.torqueLimits; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const WbIdInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const WbIdMpcRobotModel<scalar_t>& getMpcRobotModel() const { return *mpcRobotModelPtr_; }
  const WbIdMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() const { return *mpcRobotModelADPtr_; }

  const WbIdTorqueWindow& getTorqueWindow() const { return *torqueWindowPtr_; }
  //! Must be added to the solver (addSynchronizedModule): it tells the torque window
  //! where each horizon starts.
  std::shared_ptr<SolverSynchronizedModule> getTorqueWindowModule() const { return torqueWindowModulePtr_; }

 private:
  void setupOptimalControlProblem();

  ModelSettings modelSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  std::unique_ptr<WbIdMpcRobotModel<scalar_t>> mpcRobotModelPtr_;
  std::unique_ptr<WbIdMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;

  std::unique_ptr<WbIdInitializer> initializerPtr_;

  std::shared_ptr<WbIdTorqueWindow> torqueWindowPtr_;
  std::shared_ptr<SolverSynchronizedModule> torqueWindowModulePtr_;

  vector_t initialState_;
  Config config_;
  bool verbose_;
};

}  // namespace ocs2::humanoid
