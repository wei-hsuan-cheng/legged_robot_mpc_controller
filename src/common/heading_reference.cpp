#include "legged_robot_mpc_controller/common/heading_reference.hpp"

#include <algorithm>
#include <cmath>

namespace legged_robot_mpc_controller::common
{

void HeadingReference::apply(
  double yaw_rate_command,
  double init_time,
  double measured_yaw,
  ocs2::TargetTrajectories& target_trajectories,
  int yaw_state_index)
{
  if (!initialized_) {
    heading_ = measured_yaw;
    time_ = init_time;
    initialized_ = true;
  }

  heading_ += yaw_rate_command * (init_time - time_);
  time_ = init_time;

  // Anti-windup: never demand more than kMaxHeadingError of correction at once, and
  // keep the reference continuous with the measured yaw (no +-pi jumps).
  constexpr double kMaxHeadingError = 0.3;
  double error = std::remainder(heading_ - measured_yaw, 2.0 * M_PI);
  error = std::clamp(error, -kMaxHeadingError, kMaxHeadingError);
  heading_ = measured_yaw + error;

  for (std::size_t i = 0; i < target_trajectories.stateTrajectory.size(); ++i) {
    target_trajectories.stateTrajectory[i][yaw_state_index] =
      heading_ + yaw_rate_command * (target_trajectories.timeTrajectory[i] - init_time);
  }
}

}  // namespace legged_robot_mpc_controller::common
