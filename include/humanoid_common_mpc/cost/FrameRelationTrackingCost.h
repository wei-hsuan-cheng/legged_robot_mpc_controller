#ifndef HUMANOID_COMMON_MPC__COST__FRAME_RELATION_TRACKING_COST_H_
#define HUMANOID_COMMON_MPC__COST__FRAME_RELATION_TRACKING_COST_H_

#include <string>

#include "humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid
{

/**
 * Task-space tracking of a robot frame to an externally commanded world-frame
 * pose (command_type "frame_relation"), mirroring mpc_controllers'
 * FrameRelationTrackingCost but built on the package's CppAD end-effector
 * kinematics: the commanded pose enters through the runtime parameter vector of
 * EndEffectorKinematicsQuadraticCost, so the generated model is shared with the
 * task-space costs and no extra code generation per command is needed.
 *
 * The cost is inactive until the reference manager holds an external
 * frame-relation entry naming this frame. The commanded trajectory states are
 * [position(3), quaternion x y z w]; desired velocities are zero, so the
 * velocity weight entries act as damping. Command weights (6: position xyz,
 * orientation xyz) override the configured defaults when provided.
 *
 * Current scope: target frame is the world (absolute pose tracking). Tracking
 * relative to another robot frame is a future extension.
 */
class FrameRelationTrackingCost final : public EndEffectorKinematicsQuadraticCost
{
public:
  FrameRelationTrackingCost(
    EndEffectorKinematicsWeights default_weights,
    const PinocchioInterface& pinocchio_interface,
    const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
    const MpcRobotModelBase<ad_scalar_t>& mpc_robot_model_ad,
    const std::string& frame_name,
    const ModelSettings& model_settings,
    const SwitchedModelReferenceManager& reference_manager);

  ~FrameRelationTrackingCost() override = default;
  FrameRelationTrackingCost* clone() const override { return new FrameRelationTrackingCost(*this); }

  bool isActive(scalar_t time) const override;

  vector_t getParameters(
    scalar_t time,
    const TargetTrajectories& target_trajectories,
    const PreComputation& pre_computation) const override;

private:
  FrameRelationTrackingCost(const FrameRelationTrackingCost& other) = default;

  /// Index of this frame in the current external command, or -1 when absent.
  int findCommandIndex() const;

  std::string frame_name_;
  vector12_t default_sqrt_weights_;
  const SwitchedModelReferenceManager* reference_manager_ptr_;
};

}  // namespace ocs2::humanoid

#endif  // HUMANOID_COMMON_MPC__COST__FRAME_RELATION_TRACKING_COST_H_
