#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__WALKING_VELOCITY_TARGET_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__WALKING_VELOCITY_TARGET_HPP_

#include <ocs2_msgs/msg/walking_velocity_command.hpp>

#include <humanoid_common_mpc/command/WalkingVelocityCommand.h>

namespace legged_robot_mpc_controller::target
{

/** Converts the ROS walking command into the bounded internal MPC target. */
ocs2::humanoid::WalkingVelocityCommand from_message(
  const ocs2_msgs::msg::WalkingVelocityCommand& message);

}  // namespace legged_robot_mpc_controller::target

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__WALKING_VELOCITY_TARGET_HPP_
