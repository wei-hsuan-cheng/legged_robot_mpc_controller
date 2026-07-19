#ifndef HUMANOID_COMMON_MPC__COST__FRAME_RELATION_TRACKING_COST_H_
#define HUMANOID_COMMON_MPC__COST__FRAME_RELATION_TRACKING_COST_H_

#include <memory>
#include <string>

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid
{

/**
 * Task-space tracking of the relative pose of a target frame expressed in a
 * source frame (command_type "frame_relation"), mirroring mpc_controllers'
 * FrameRelationTrackingCost with the package's CppAD kinematics.
 *
 * Convention (matching mpc_controllers): the SOURCE frame is the reference
 * (root) the pose is expressed in — a robot frame like "pelvis" or a global
 * frame ("world"/"odom"/"map"/"global"); the TARGET frame is the tracked leaf
 * (e.g. a hand). The commanded trajectory states are
 * [position(3), quaternion x y z w] of target in source.
 *
 * The frame pair is fixed at construction (the CppAD model contains both
 * forward kinematics); the commanded pose and weights enter through the runtime
 * parameter vector (25 = reference pose/twist 13 + sqrt weights 12), so
 * commands never trigger code regeneration. The cost is inactive until an
 * external frame-relation command names this exact pair. Desired relative
 * velocities are zero, so the velocity weight entries act as damping. Command
 * weights (6: position xyz, orientation xyz) override the configured defaults.
 */
class FrameRelationTrackingCost final : public StateInputCostGaussNewtonAd
{
public:
  FrameRelationTrackingCost(
    EndEffectorKinematicsWeights default_weights,
    const PinocchioInterface& pinocchio_interface,
    const MpcRobotModelBase<ad_scalar_t>& mpc_robot_model_ad,
    const std::string& source_frame,
    const std::string& target_frame,
    const ModelSettings& model_settings,
    const SwitchedModelReferenceManager& reference_manager);

  ~FrameRelationTrackingCost() override = default;
  FrameRelationTrackingCost* clone() const override { return new FrameRelationTrackingCost(*this); }

  bool isActive(scalar_t time) const override;

  vector_t getParameters(
    scalar_t time,
    const TargetTrajectories& target_trajectories,
    const PreComputation& pre_computation) const override;

  static bool isGlobalFrameName(const std::string& frame)
  {
    return frame == "world" || frame == "odom" || frame == "map" || frame == "global";
  }

protected:
  FrameRelationTrackingCost(const FrameRelationTrackingCost& other);

  ad_vector_t costVectorFunction(
    ad_scalar_t time,
    const ad_vector_t& state,
    const ad_vector_t& input,
    const ad_vector_t& parameters) const override;

private:
  /// Index of this pair in the current external command, or -1 when absent.
  int findCommandIndex() const;

  std::string source_frame_;
  std::string target_frame_;
  bool source_is_global_{false};
  pinocchio::FrameIndex source_frame_id_{0};
  pinocchio::FrameIndex target_frame_id_{0};
  vector12_t default_sqrt_weights_;
  mutable PinocchioInterfaceCppAd pinocchio_interface_cppad_;
  std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpc_robot_model_ad_ptr_;
  const SwitchedModelReferenceManager* reference_manager_ptr_;
};

}  // namespace ocs2::humanoid

#endif  // HUMANOID_COMMON_MPC__COST__FRAME_RELATION_TRACKING_COST_H_
