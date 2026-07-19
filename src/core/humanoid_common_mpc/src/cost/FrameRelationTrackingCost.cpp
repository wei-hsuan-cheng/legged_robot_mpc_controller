#include "humanoid_common_mpc/cost/FrameRelationTrackingCost.h"

#include <stdexcept>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2::humanoid
{

FrameRelationTrackingCost::FrameRelationTrackingCost(
  EndEffectorKinematicsWeights default_weights,
  const PinocchioInterface& pinocchio_interface,
  const MpcRobotModelBase<ad_scalar_t>& mpc_robot_model_ad,
  const std::string& source_frame,
  const std::string& target_frame,
  const ModelSettings& model_settings,
  const SwitchedModelReferenceManager& reference_manager)
: StateInputCostGaussNewtonAd(),
  source_frame_(source_frame),
  target_frame_(target_frame),
  source_is_global_(isGlobalFrameName(source_frame)),
  default_sqrt_weights_(default_weights.toVector().cwiseSqrt()),
  pinocchio_interface_cppad_(pinocchio_interface.toCppAd()),
  mpc_robot_model_ad_ptr_(mpc_robot_model_ad.clone()),
  reference_manager_ptr_(&reference_manager)
{
  const auto& model = pinocchio_interface.getModel();
  if (!source_is_global_) {
    if (!model.existFrame(source_frame_)) {
      throw std::invalid_argument(
        "[FrameRelationTrackingCost] unknown source frame '" + source_frame_ + "'");
    }
    source_frame_id_ = model.getFrameId(source_frame_);
  }
  if (!model.existFrame(target_frame_)) {
    throw std::invalid_argument(
      "[FrameRelationTrackingCost] unknown target frame '" + target_frame_ + "'");
  }
  target_frame_id_ = model.getFrameId(target_frame_);

  std::cout << "Initialized FrameRelationTrackingCost " << source_frame_ << " -> " << target_frame_
            << " with default weights: " << default_weights.toVector().transpose() << std::endl;

  initialize(
    mpc_robot_model_ad_ptr_->getStateDim(), mpc_robot_model_ad_ptr_->getInputDim(), 25,
    source_frame_ + "_to_" + target_frame_ + "_FrameRelationTrackingCost",
    model_settings.modelFolderCppAd, model_settings.recompileLibrariesCppAd,
    model_settings.verboseCppAd);
}

FrameRelationTrackingCost::FrameRelationTrackingCost(const FrameRelationTrackingCost& other)
: StateInputCostGaussNewtonAd(other),
  source_frame_(other.source_frame_),
  target_frame_(other.target_frame_),
  source_is_global_(other.source_is_global_),
  source_frame_id_(other.source_frame_id_),
  target_frame_id_(other.target_frame_id_),
  default_sqrt_weights_(other.default_sqrt_weights_),
  pinocchio_interface_cppad_(other.pinocchio_interface_cppad_),
  mpc_robot_model_ad_ptr_(other.mpc_robot_model_ad_ptr_->clone()),
  reference_manager_ptr_(other.reference_manager_ptr_)
{
}

int FrameRelationTrackingCost::findCommandIndex() const
{
  const auto& command = reference_manager_ptr_->getExternalFrameRelationTargets();
  for (size_t i = 0; i < command.sourceFrames.size(); ++i) {
    if (command.sourceFrames[i] == source_frame_ && command.targetFrames[i] == target_frame_) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool FrameRelationTrackingCost::isActive(scalar_t /*time*/) const
{
  return findCommandIndex() >= 0;
}

vector_t FrameRelationTrackingCost::getParameters(
  scalar_t time,
  const TargetTrajectories& /*target_trajectories*/,
  const PreComputation& /*pre_computation*/) const
{
  const int index = findCommandIndex();
  if (index < 0) {
    // Inactive; return a well-formed zero-weight parameter vector.
    vector_t parameters = vector_t::Zero(25);
    parameters(6) = 1.0;  // identity quaternion w
    return parameters;
  }

  const auto& command = reference_manager_ptr_->getExternalFrameRelationTargets();
  const vector_t pose = command.targets[static_cast<size_t>(index)].getDesiredState(time);
  if (pose.size() != 7) {
    throw std::runtime_error(
      "[FrameRelationTrackingCost] commanded pose state must be [position(3), quaternion xyzw]");
  }

  EndEffectorKinematicsCostElement<scalar_t> reference;
  reference.setPosition(pose.head<3>());
  quaternion_t quaternion(pose(6), pose(3), pose(4), pose(5));  // w, x, y, z
  reference.setOrientation(quaternion.normalized());
  reference.setLinearVelocity(vector3_t::Zero());
  reference.setAngularVelocity(vector3_t::Zero());

  vector12_t sqrt_weights = default_sqrt_weights_;
  const vector_t& command_weights = command.weights[static_cast<size_t>(index)];
  if (command_weights.size() == 6) {
    sqrt_weights.head<6>() = command_weights.cwiseMax(0.0).cwiseSqrt();
  }

  vector_t parameters(25);
  parameters << reference.getValues(), sqrt_weights;
  return parameters;
}

ad_vector_t FrameRelationTrackingCost::costVectorFunction(
  ad_scalar_t /*time*/,
  const ad_vector_t& state,
  const ad_vector_t& input,
  const ad_vector_t& parameters) const
{
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const auto& model = pinocchio_interface_cppad_.getModel();
  auto& data = pinocchio_interface_cppad_.getData();

  const ad_vector_t q = mpc_robot_model_ad_ptr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpc_robot_model_ad_ptr_->getGeneralizedVelocities(state, input);
  pinocchio::forwardKinematics(model, data, q, v);

  const auto target_placement = pinocchio::updateFramePlacement(model, data, target_frame_id_);
  const ad_vector_t target_position = target_placement.translation();
  const Eigen::Matrix<ad_scalar_t, 3, 3> target_rotation = target_placement.rotation();
  const auto target_velocity = pinocchio::getFrameVelocity(model, data, target_frame_id_, rf);

  ad_vector_t relative_position(3);
  Eigen::Matrix<ad_scalar_t, 3, 3> relative_rotation;
  ad_vector_t relative_linear_velocity(3);
  ad_vector_t relative_angular_velocity(3);

  if (source_is_global_) {
    relative_position = target_position;
    relative_rotation = target_rotation;
    relative_linear_velocity = target_velocity.linear();
    relative_angular_velocity = target_velocity.angular();
  } else {
    const auto source_placement = pinocchio::updateFramePlacement(model, data, source_frame_id_);
    const ad_vector_t source_position = source_placement.translation();
    const Eigen::Matrix<ad_scalar_t, 3, 3> source_rotation = source_placement.rotation();
    const auto source_velocity = pinocchio::getFrameVelocity(model, data, source_frame_id_, rf);

    const Eigen::Matrix<ad_scalar_t, 3, 3> source_rotation_transpose = source_rotation.transpose();
    const ad_vector_t position_offset = target_position - source_position;
    relative_position = source_rotation_transpose * position_offset;
    relative_rotation = source_rotation_transpose * target_rotation;

    // Relative twist as observed from the (moving) source frame.
    const Eigen::Matrix<ad_scalar_t, 3, 1> source_angular = source_velocity.angular();
    const Eigen::Matrix<ad_scalar_t, 3, 1> offset3 = position_offset;
    relative_linear_velocity = source_rotation_transpose *
      (target_velocity.linear() - source_velocity.linear() - source_angular.cross(offset3));
    relative_angular_velocity =
      source_rotation_transpose * (target_velocity.angular() - source_angular);
  }

  const ad_quaternion_t relative_orientation = matrixToQuaternion(relative_rotation);

  Eigen::Matrix<ad_scalar_t, 13, 1> task_space_vector;
  task_space_vector << relative_position, relative_orientation.coeffs(), relative_linear_velocity,
    relative_angular_velocity;

  const ad_vector_t errors = computeTaskSpaceErrors(
    EndEffectorKinematicsCostElement<ad_scalar_t>(task_space_vector),
    EndEffectorKinematicsCostElement<ad_scalar_t>(parameters.head(13)));

  const ad_vector_t sqrt_weight_parameters = parameters.segment<12>(13);
  return errors.cwiseProduct(sqrt_weight_parameters);
}

}  // namespace ocs2::humanoid
