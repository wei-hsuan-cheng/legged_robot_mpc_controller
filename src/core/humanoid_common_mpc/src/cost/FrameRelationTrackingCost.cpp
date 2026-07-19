#include "humanoid_common_mpc/cost/FrameRelationTrackingCost.h"

#include <stdexcept>

namespace ocs2::humanoid
{

FrameRelationTrackingCost::FrameRelationTrackingCost(
  EndEffectorKinematicsWeights default_weights,
  const PinocchioInterface& pinocchio_interface,
  const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
  const MpcRobotModelBase<ad_scalar_t>& mpc_robot_model_ad,
  const std::string& frame_name,
  const ModelSettings& model_settings,
  const SwitchedModelReferenceManager& reference_manager)
: EndEffectorKinematicsQuadraticCost(
    default_weights, pinocchio_interface, end_effector_kinematics, mpc_robot_model_ad,
    frame_name, model_settings),
  frame_name_(frame_name),
  default_sqrt_weights_(default_weights.toVector().cwiseSqrt()),
  reference_manager_ptr_(&reference_manager)
{
}

int FrameRelationTrackingCost::findCommandIndex() const
{
  const auto& command = reference_manager_ptr_->getExternalFrameRelationTargets();
  for (size_t i = 0; i < command.sourceFrames.size(); ++i) {
    if (command.sourceFrames[i] == frame_name_) {
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

}  // namespace ocs2::humanoid
