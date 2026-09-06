#include "humanoid_common_mpc/cost/BaseMotionTrackingCost.h"

#include <cmath>
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

BaseMotionTrackingCost::DeviationLinearization
BaseMotionTrackingCost::getDeviationLinearization(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories) const
{
  const vector_t desired_state = reference_manager_ptr_->getDesiredState(
    target_trajectories, state, time);

  DeviationLinearization result{
    vector_t::Zero(12),
    matrix_t::Zero(12, mpc_robot_model_ptr_->getStateDim())};

  const vector6_t base_pose = mpc_robot_model_ptr_->getBasePose(state);
  const vector6_t desired_pose = mpc_robot_model_ptr_->getBasePose(desired_state);
  const vector6_t base_motion = mpc_robot_model_ptr_->getBaseComVelocity(state);
  const vector6_t desired_motion = mpc_robot_model_ptr_->getBaseComVelocity(desired_state);
  const auto pose_index = static_cast<Eigen::Index>(mpc_robot_model_ptr_->getBaseStartindex());
  const auto motion_index = static_cast<Eigen::Index>(
    mpc_robot_model_ptr_->getBaseComVelocityStartindex());

  const scalar_t cos_yaw = std::cos(desired_pose(3));
  const scalar_t sin_yaw = std::sin(desired_pose(3));
  matrix_t world_to_reference = matrix_t::Zero(2, 2);
  world_to_reference << cos_yaw, sin_yaw, -sin_yaw, cos_yaw;

  // x, y, and yaw are gauge variables for proprioceptive locomotion. In
  // relative-twist modes the optimizer tracks commanded motion without trying
  // to restore the estimator's drifting world origin or heading. Explicit
  // base-pose commands retain full pose tracking.
  if (!reference_manager_ptr_->usesRelativeBaseTwistTracking()) {
    result.deviation.head<2>() =
      world_to_reference * (base_pose.head<2>() - desired_pose.head<2>());
    result.jacobian.block<2, 2>(0, pose_index) = world_to_reference;
    result.deviation(3) = std::remainder(base_pose(3) - desired_pose(3), 2.0 * M_PI);
    result.jacobian(3, pose_index + 3) = 1.0;
  }

  result.deviation(2) = base_pose(2) - desired_pose(2);
  result.jacobian(2, pose_index + 2) = 1.0;
  for (Eigen::Index index = 4; index < 6; ++index) {
    result.deviation(index) =
      std::remainder(base_pose(index) - desired_pose(index), 2.0 * M_PI);
    result.jacobian(index, pose_index + index) = 1.0;
  }

  // Compare horizontal velocity (or normalized linear momentum in the
  // centroidal model) in the corresponding pelvis frames. A common yaw-gauge
  // change rotates each world-frame vector and its pose together, leaving this
  // error unchanged.
  const scalar_t cos_base_yaw = std::cos(base_pose(3));
  const scalar_t sin_base_yaw = std::sin(base_pose(3));
  matrix_t world_to_base = matrix_t::Zero(2, 2);
  world_to_base << cos_base_yaw, sin_base_yaw, -sin_base_yaw, cos_base_yaw;
  const vector2_t base_motion_local = world_to_base * base_motion.head<2>();
  const vector2_t desired_motion_local = world_to_reference * desired_motion.head<2>();
  result.deviation.segment<2>(6) = base_motion_local - desired_motion_local;
  result.jacobian.block<2, 2>(6, motion_index) = world_to_base;
  result.jacobian(6, pose_index + 3) = base_motion_local(1);
  result.jacobian(7, pose_index + 3) = -base_motion_local(0);
  result.deviation.segment<4>(8) = base_motion.tail<4>() - desired_motion.tail<4>();
  result.jacobian.block<4, 4>(8, motion_index + 2).setIdentity();

  return result;
}

scalar_t BaseMotionTrackingCost::getValue(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories,
  const PreComputation&) const
{
  const auto linearization = getDeviationLinearization(time, state, target_trajectories);
  return 0.5 * linearization.deviation.dot(gains_ * linearization.deviation);
}

ScalarFunctionQuadraticApproximation BaseMotionTrackingCost::getQuadraticApproximation(
  scalar_t time,
  const vector_t& state,
  const TargetTrajectories& target_trajectories,
  const PreComputation&) const
{
  const auto linearization = getDeviationLinearization(time, state, target_trajectories);

  ScalarFunctionQuadraticApproximation approximation;
  approximation.f =
    0.5 * linearization.deviation.dot(gains_ * linearization.deviation);
  approximation.dfdx =
    linearization.jacobian.transpose() * gains_ * linearization.deviation;
  approximation.dfdxx =
    linearization.jacobian.transpose() * gains_ * linearization.jacobian;
  return approximation;
}

}  // namespace ocs2::humanoid
