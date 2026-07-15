#include "humanoid_common_mpc/cost/BaseMotionTrackingCost.h"

#include <stdexcept>
#include <utility>

namespace ocs2::humanoid
{

BaseMotionTrackingCost::BaseMotionTrackingCost(
  matrix_t gains,
  const SwitchedModelReferenceManager& reference_manager,
  const MpcRobotModelBase<scalar_t>& mpc_robot_model)
: gains_(std::move(gains)),
  reference_manager_ptr_(&reference_manager),
  mpc_robot_model_ptr_(&mpc_robot_model)
{
  if (gains_.rows() != 12 || gains_.cols() != 12) {
    throw std::invalid_argument(
      "[BaseMotionTrackingCost] gains must be a 12x12 matrix for base pose and motion");
  }
}

vector_t BaseMotionTrackingCost::getDeviation(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories) const
{
  const vector_t desired_state = reference_manager_ptr_->getDesiredState(
    target_trajectories, state, time);
  vector_t deviation(12);
  deviation.head<6>() =
    mpc_robot_model_ptr_->getBasePose(state) - mpc_robot_model_ptr_->getBasePose(desired_state);
  deviation.tail<6>() =
    mpc_robot_model_ptr_->getBaseComVelocity(state) -
    mpc_robot_model_ptr_->getBaseComVelocity(desired_state);
  return deviation;
}

matrix_t BaseMotionTrackingCost::getStateJacobian() const
{
  matrix_t jacobian = matrix_t::Zero(12, mpc_robot_model_ptr_->getStateDim());
  jacobian.block<6, 6>(0, mpc_robot_model_ptr_->getBaseStartindex()).setIdentity();
  jacobian.block<6, 6>(
    6, mpc_robot_model_ptr_->getBaseComVelocityStartindex()).setIdentity();
  return jacobian;
}

scalar_t BaseMotionTrackingCost::getValue(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories,
  const PreComputation&) const
{
  const vector_t deviation = getDeviation(time, state, target_trajectories);
  return 0.5 * deviation.dot(gains_ * deviation);
}

ScalarFunctionQuadraticApproximation BaseMotionTrackingCost::getQuadraticApproximation(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories,
  const PreComputation&) const
{
  const vector_t deviation = getDeviation(time, state, target_trajectories);
  const matrix_t jacobian = getStateJacobian();

  ScalarFunctionQuadraticApproximation approximation;
  approximation.f = 0.5 * deviation.dot(gains_ * deviation);
  approximation.dfdx = jacobian.transpose() * gains_ * deviation;
  approximation.dfdxx = jacobian.transpose() * gains_ * jacobian;
  return approximation;
}

}  // namespace ocs2::humanoid
