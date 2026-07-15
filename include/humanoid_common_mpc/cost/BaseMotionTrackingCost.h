#ifndef HUMANOID_COMMON_MPC__COST__BASE_MOTION_TRACKING_COST_H_
#define HUMANOID_COMMON_MPC__COST__BASE_MOTION_TRACKING_COST_H_

#include <ocs2_core/cost/StateCost.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid
{

/**
 * Quadratic tracking cost for the model's base pose and base motion state.
 *
 * The model-specific state layout is accessed through MpcRobotModelBase, so
 * the same cost can be used by the whole-body and centroidal formulations.
 */
class BaseMotionTrackingCost final : public StateCost
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BaseMotionTrackingCost(
    matrix_t gains,
    const SwitchedModelReferenceManager& reference_manager,
    const MpcRobotModelBase<scalar_t>& mpc_robot_model);

  BaseMotionTrackingCost* clone() const override
  {
    return new BaseMotionTrackingCost(*this);
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
  BaseMotionTrackingCost(const BaseMotionTrackingCost& other) = default;

  vector_t getDeviation(
    scalar_t time,
    const vector_t& state,
    const TargetTrajectories& target_trajectories) const;

  matrix_t getStateJacobian() const;

  matrix_t gains_;
  const SwitchedModelReferenceManager* reference_manager_ptr_;
  const MpcRobotModelBase<scalar_t>* mpc_robot_model_ptr_;
};

}  // namespace ocs2::humanoid

#endif  // HUMANOID_COMMON_MPC__COST__BASE_MOTION_TRACKING_COST_H_
