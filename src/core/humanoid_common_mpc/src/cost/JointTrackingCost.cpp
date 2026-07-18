#include "humanoid_common_mpc/cost/JointTrackingCost.h"

#include <stdexcept>
#include <utility>

namespace ocs2::humanoid
{

JointTrackingCost::JointTrackingCost(
  matrix_t gains,
  std::vector<size_t> joint_block_indices,
  const SwitchedModelReferenceManager& reference_manager,
  const MpcRobotModelBase<scalar_t>& mpc_robot_model,
  bool apply_arm_swing_reference)
: gains_(std::move(gains)),
  joint_block_indices_(std::move(joint_block_indices)),
  reference_manager_ptr_(&reference_manager),
  mpc_robot_model_ptr_(&mpc_robot_model),
  apply_arm_swing_reference_(apply_arm_swing_reference)
{
  const auto n = static_cast<Eigen::Index>(joint_block_indices_.size());
  if (gains_.rows() != n || gains_.cols() != n) {
    throw std::invalid_argument(
      "[JointTrackingCost] gains must be a square matrix matching the number of tracked joints");
  }
}

vector_t JointTrackingCost::getDesiredState(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories) const
{
  // Running term tracks the arm-swing reference; the terminal term tracks the
  // raw posture reference, matching the original StateInput/terminal split.
  if (apply_arm_swing_reference_) {
    return reference_manager_ptr_->getDesiredState(target_trajectories, state, time);
  }
  return target_trajectories.getDesiredState(time);
}

vector_t JointTrackingCost::getDeviation(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories) const
{
  const auto n = static_cast<Eigen::Index>(joint_block_indices_.size());
  const vector_t joints = mpc_robot_model_ptr_->getJointAngles(state);
  vector_t deviation(n);

  // Command-type "joint": an external target trajectory (states in tracked-joint
  // order) overrides the internal posture/arm-swing reference.
  if (reference_manager_ptr_->hasExternalJointTargets()) {
    const vector_t desired_joints =
      reference_manager_ptr_->getExternalJointTargets().getDesiredState(time);
    if (desired_joints.size() != n) {
      throw std::runtime_error(
        "[JointTrackingCost] external joint target state size mismatch with tracked joints");
    }
    for (Eigen::Index i = 0; i < n; ++i) {
      deviation(i) = joints(static_cast<Eigen::Index>(joint_block_indices_[i])) - desired_joints(i);
    }
    return deviation;
  }

  const vector_t desired_state = getDesiredState(time, state, target_trajectories);
  const vector_t desired_joints = mpc_robot_model_ptr_->getJointAngles(desired_state);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto j = static_cast<Eigen::Index>(joint_block_indices_[i]);
    deviation(i) = joints(j) - desired_joints(j);
  }
  return deviation;
}

matrix_t JointTrackingCost::getStateJacobian() const
{
  const auto n = static_cast<Eigen::Index>(joint_block_indices_.size());
  const auto joint_start = static_cast<Eigen::Index>(mpc_robot_model_ptr_->getJointStartindex());
  matrix_t jacobian = matrix_t::Zero(n, mpc_robot_model_ptr_->getStateDim());
  for (Eigen::Index i = 0; i < n; ++i) {
    jacobian(i, joint_start + static_cast<Eigen::Index>(joint_block_indices_[i])) = 1.0;
  }
  return jacobian;
}

scalar_t JointTrackingCost::getValue(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories,
  const PreComputation&) const
{
  const vector_t deviation = getDeviation(time, state, target_trajectories);
  return 0.5 * deviation.dot(gains_ * deviation);
}

ScalarFunctionQuadraticApproximation JointTrackingCost::getQuadraticApproximation(
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
