#include "legged_robot_mpc_controller/common/target/walking_velocity_target.hpp"

#include <algorithm>

namespace legged_robot_mpc_controller::target
{

ocs2::humanoid::WalkingVelocityCommand from_message(
  const ocs2_msgs::msg::WalkingVelocityCommand& message)
{
  ocs2::humanoid::WalkingVelocityCommand command;
  command.linear_velocity_x = std::clamp(message.linear_velocity_x, -1.0, 1.0);
  command.linear_velocity_y = std::clamp(message.linear_velocity_y, -1.0, 1.0);
  command.desired_pelvis_height = std::clamp(message.desired_pelvis_height, 0.2, 1.0);
  command.angular_velocity_z = std::clamp(message.angular_velocity_z, -1.0, 1.0);
  return command;
}

}  // namespace legged_robot_mpc_controller::target
