#ifndef HUMANOID_COMMON_MPC__COST__JOINT_TRACKING_COST_H_
#define HUMANOID_COMMON_MPC__COST__JOINT_TRACKING_COST_H_

#include <vector>

#include <ocs2_core/cost/StateCost.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid
{

/**
 * Quadratic tracking cost for a configured subset of the MPC joints (the arm
 * joints), mirroring the JointTracking term of the mpc_controllers stack.
 *
 * Structure (target + cost) so a Cartesian frame-relation tracking term can be
 * added alongside it later: the desired joint reference is read from the
 * SwitchedModelReferenceManager, and the model-specific state layout is accessed
 * through MpcRobotModelBase so the same cost serves the whole-body and
 * centroidal formulations.
 *
 * The tracked joints are a subset of the joint block, addressed by their index
 * within that block (0 .. mpc_joint_dim-1). Because joint-position tracking is a
 * linear function of the state, the Jacobian is a constant selection matrix and
 * is evaluated analytically (no auto-diff needed).
 *
 * @param apply_arm_swing_reference selects the desired reference:
 *   true  -> SwitchedModelReferenceManager::getDesiredState (posture + gait arm
 *            swing), matching the running StateInputQuadraticCost;
 *   false -> the raw TargetTrajectories desired state (posture only), matching
 *            the terminal QuadraticStateCost.
 */
class JointTrackingCost final : public StateCost
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  JointTrackingCost(
    matrix_t gains,
    std::vector<size_t> joint_block_indices,
    const SwitchedModelReferenceManager& reference_manager,
    const MpcRobotModelBase<scalar_t>& mpc_robot_model,
    bool apply_arm_swing_reference);

  JointTrackingCost* clone() const override
  {
    return new JointTrackingCost(*this);
  }

  scalar_t getValue(
    scalar_t time,
    const vector_t& state,
    const TargetTrajectories& target_trajectories,
    const PreComputation& pre_computation) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    scalar_t time,
    const vector_t& state,
    const TargetTrajectories& target_trajectories,
    const PreComputation& pre_computation) const override;

private:
  JointTrackingCost(const JointTrackingCost& other) = default;

  vector_t getDesiredState(
    scalar_t time,
    const vector_t& state,
    const TargetTrajectories& target_trajectories) const;

  vector_t getDeviation(
    scalar_t time,
    const vector_t& state,
    const TargetTrajectories& target_trajectories) const;

  matrix_t getStateJacobian() const;

  matrix_t gains_;
  std::vector<size_t> joint_block_indices_;
  const SwitchedModelReferenceManager* reference_manager_ptr_;
  const MpcRobotModelBase<scalar_t>* mpc_robot_model_ptr_;
  bool apply_arm_swing_reference_;
};

}  // namespace ocs2::humanoid

#endif  // HUMANOID_COMMON_MPC__COST__JOINT_TRACKING_COST_H_
