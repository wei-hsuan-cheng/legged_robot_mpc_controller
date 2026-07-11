#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__HEADING_REFERENCE_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__HEADING_REFERENCE_HPP_

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace legged_robot_mpc_controller::common
{

/// Persistent heading reference for velocity-commanded walking.
///
/// The legacy target-trajectory calculator anchors the yaw target to the measured yaw
/// every solve, so slow yaw drift is never corrected. This helper integrates the
/// commanded yaw rate into a persistent heading and overwrites the yaw targets with it
/// (with anti-windup against large disturbances). Call only from the solver thread
/// (preSolverRun path).
class HeadingReference
{
public:
  void reset() { initialized_ = false; }

  /// Overwrites the yaw entries of the target state trajectory with the integrated
  /// heading reference. `measured_yaw` is the current base yaw from the observation.
  /// `yaw_state_index` is the yaw entry in the target state (3 for the whole-body
  /// model, 9 for the centroidal model where the pose starts after the momentum).
  void apply(
    double yaw_rate_command,
    double init_time,
    double measured_yaw,
    ocs2::TargetTrajectories& target_trajectories,
    int yaw_state_index = 3);

private:
  bool initialized_{false};
  double heading_{0.0};
  double time_{0.0};
};

}  // namespace legged_robot_mpc_controller::common

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__HEADING_REFERENCE_HPP_
