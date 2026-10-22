// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_linear_kf.hpp"

#include "humanoid_linear_kf_parameters.hpp"

namespace legged_robot_mpc_controller
{

struct HumanoidLinearKf::Params
{
  std::shared_ptr<humanoid_linear_kf::ParamListener> listener;
  humanoid_linear_kf::Params values;
};

HumanoidLinearKf::HumanoidLinearKf() : HumanoidStateEstimator("humanoid_linear_kf") {}
HumanoidLinearKf::~HumanoidLinearKf() = default;

bool HumanoidLinearKf::declareParameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  try {
    params_ = std::make_unique<Params>();
    params_->listener = std::make_shared<humanoid_linear_kf::ParamListener>(node, name());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] declaring parameters failed: %s", name().c_str(), e.what());
    return false;
  }
  return true;
}

bool HumanoidLinearKf::readParameters(
  EstimatorCommonSettings & common, state_estimation::EstimatorSettings & filter)
{
  if (!params_ || !params_->listener) {
    return false;
  }
  params_->values = params_->listener->get_params();
  const auto & p = params_->values;

  common.imu_interface_name = p.imuInterfaceName;
  common.imu_frame = p.imuFrame;
  common.contact_frames = p.contactFrames;
  common.odom_topic = p.odomTopic;
  common.odom_frame = p.odomFrame;
  common.publish_rate = p.publishRate;
  common.contact_source = p.contact.source;
  common.ready_min_seconds = p.readiness.minSeconds;
  common.ready_roll_pitch_std_max = p.readiness.rollPitchStdDevMax;
  common.ready_height_std_max = p.readiness.heightStdDevMax;
  common.ready_velocity_std_max = p.readiness.velocityStdDevMax;

  filter.contact_foot_indices = p.contact.footIndices;

  filter.num_feet = static_cast<int>(p.numFeet);
  filter.lkf_foot_radius = p.footRadius;
  filter.lkf_imu_process_noise_position = p.imuProcessNoisePosition;
  filter.lkf_imu_process_noise_velocity = p.imuProcessNoiseVelocity;
  filter.lkf_foot_process_noise_position = p.footProcessNoisePosition;
  filter.lkf_foot_sensor_noise_position = p.footSensorNoisePosition;
  filter.lkf_foot_sensor_noise_velocity = p.footSensorNoiseVelocity;
  filter.lkf_foot_height_sensor_noise = p.footHeightSensorNoise;
  filter.lkf_swing_noise_multiplier = p.swingNoiseMultiplier;
  return true;
}

}  // namespace legged_robot_mpc_controller

