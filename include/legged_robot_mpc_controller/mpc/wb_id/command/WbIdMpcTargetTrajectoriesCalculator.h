#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>

#include <legged_robot_mpc_controller/mpc/common/command/TargetTrajectoriesCalculatorBase.h>
#include <legged_robot_mpc_controller/mpc/common/model/MpcRobotModelBase.h>
#include <legged_robot_mpc_controller/mpc/common/model/Types.h>

namespace ocs2::humanoid {

/**
 * Commands -> target trajectories for the whole-body state [q, v].
 *
 * Carries commanded planar velocity in world coordinates, yaw rate as a ZYX
 * Euler rate, and zero joint velocities. The pose is integrated from the current
 * pose using the conditioned command.
 */
class WbIdMpcTargetTrajectoriesCalculator : public TargetTrajectoriesCalculatorBase {
 public:
  WbIdMpcTargetTrajectoriesCalculator(const ReferenceConfig& referenceConfig,
                                      const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                      scalar_t mpcHorizon);

  WbIdMpcTargetTrajectoriesCalculator(const WbIdMpcTargetTrajectoriesCalculator& rhs) = delete;

  TargetTrajectories commandedPositionToTargetTrajectories(const vector4_t& commandLineTarget,
                                                           scalar_t initTime,
                                                           const vector_t& initState) override;

  TargetTrajectories commandedBasePoseToTargetTrajectories(const vector6_t& targetBasePose,
                                                           scalar_t initTime,
                                                           const vector_t& initState) override;

  TargetTrajectories commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                           scalar_t initTime,
                                                           const vector_t& initState) override;

 private:
  vector_t makeState(const vector6_t& basePose, const vector6_t& baseVelocity) const;
};

}  // namespace ocs2::humanoid
