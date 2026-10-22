#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>

#include <humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <humanoid_common_mpc/common/Types.h>

namespace ocs2::humanoid {

/**
 * Commands -> target trajectories for the whole-body state [q, v].
 *
 * Same references as CentroidalMpcTargetTrajectoriesCalculator, expressed in the
 * whole-body state: where the centroidal state carries normalized momentum, this one
 * carries the base twist directly - the commanded planar velocity in world coordinates
 * and the yaw rate as the ZYX Euler-rate (which IS the yaw rate at zero pitch/roll) -
 * and zero joint velocities. The pose is integrated from the current pose exactly as in
 * the centroidal case, so the two MPCs see the same commanded motion.
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
