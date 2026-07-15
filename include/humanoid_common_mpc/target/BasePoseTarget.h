#pragma once

#include <functional>
#include <mutex>

#include <ocs2_core/reference/TargetTrajectories.h>

#include "humanoid_common_mpc/command/BasePoseCommand.h"
#include "humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

namespace ocs2::humanoid {

/**
 * Thread-safe absolute base-pose target.
 *
 * The ROS callback only stores a position/quaternion command. evaluate() runs
 * on the solver thread, converts the quaternion to the MPC Euler-ZYX state,
 * unwraps the angles around the current state, and builds model-specific
 * TargetTrajectories through the injected generator.
 */
class BasePoseTarget {
 public:
  using Generator = std::function<TargetTrajectories(
      const vector6_t& targetBasePose, scalar_t initTime, scalar_t finalTime, const vector_t& initState)>;

  struct Output {
    TargetTrajectories targetTrajectories;
    vector4_t motionCommand;  //!< physical [v_x_local, v_y_local, pelvis_height, yaw_rate]
  };

  BasePoseTarget(const ReferenceConfig& referenceConfig,
                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                 Generator generator);

  void setCommand(const BasePoseCommand& command);

  Output evaluate(scalar_t initTime, scalar_t finalTime, const vector_t& initState) const;

 private:
  Generator generator_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  scalar_t targetDisplacementVelocity_;
  scalar_t targetRotationVelocity_;
  scalar_t maxDisplacementVelocityX_;
  scalar_t maxDisplacementVelocityY_;
  scalar_t positionTolerance_;
  scalar_t orientationTolerance_;

  mutable std::mutex commandMutex_;
  BasePoseCommand command_;
  bool commandReceived_{false};
};

}  // namespace ocs2::humanoid
