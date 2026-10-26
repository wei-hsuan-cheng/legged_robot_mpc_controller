// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_inekf.hpp"

#include "humanoid_inekf_parameters.hpp"

namespace legged_robot_mpc_controller
{

struct HumanoidInEkf::Params
{
  std::shared_ptr<humanoid_inekf::ParamListener> listener;
  humanoid_inekf::Params values;
};

HumanoidInEkf::HumanoidInEkf() : HumanoidStateEstimator("humanoid_inekf") {}
HumanoidInEkf::~HumanoidInEkf() = default;

bool HumanoidInEkf::declareParameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  try {
    params_ = std::make_unique<Params>();
    params_->listener = std::make_shared<humanoid_inekf::ParamListener>(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] declaring parameters failed: %s", name().c_str(), e.what());
    return false;
  }
  return true;
}

bool HumanoidInEkf::readParameters(
  EstimatorCommonSettings & common, state_estimation::EstimatorSettings & filter)
{
  if (!params_ || !params_->listener) {
    return false;
  }
  params_->values = params_->listener->get_params();
  const auto & p = params_->values;

  common.urdf_file = p.urdfFile;
  common.joint_names = p.jointNames;
  common.base_frame = p.baseFrame;
  common.imu_interface_name = p.imuInterfaceName;
  common.imu_frame = p.imuFrame;
  common.contact_frames = p.contactFrames;
  common.odom_topic = p.odomTopic;
  common.odom_frame = p.odomFrame;
  common.publish_rate = p.publishRate;
  common.contact_source = p.contact.source;
  common.contact_sensor_interfaces = p.contact.sensorInterfaces;
  common.ready_min_seconds = p.readiness.minSeconds;
  common.ready_roll_pitch_std_max = p.readiness.rollPitchStdDevMax;
  common.ready_height_std_max = p.readiness.heightStdDevMax;
  common.ready_velocity_std_max = p.readiness.velocityStdDevMax;


  filter.gyroscope_noise = p.noise.gyroscope;
  filter.accelerometer_noise = p.noise.accelerometer;
  filter.gyroscope_bias_noise = p.noise.gyroscopeBias;
  filter.accelerometer_bias_noise = p.noise.accelerometerBias;
  filter.contact_noise = p.noise.contact;
  filter.contact_position_noise = p.noise.contactPosition;
  filter.contact_rotation_noise = p.noise.contactRotation;

  filter.initial_attitude_noise = p.initialCovariance.attitude;
  filter.initial_velocity_noise = p.initialCovariance.velocity;
  filter.initial_position_noise = p.initialCovariance.position;
  filter.initial_gyroscope_bias_noise = p.initialCovariance.gyroscopeBias;
  filter.initial_accelerometer_bias_noise = p.initialCovariance.accelerometerBias;
  filter.estimate_imu_bias = p.estimateImuBias;

  filter.contact_beta0 = p.contact.beta0;
  filter.contact_beta1 = p.contact.beta1;
  filter.contact_force_covariance_alpha = p.contact.forceCovarianceAlpha;
  filter.contact_probability_threshold = p.contact.probabilityThreshold;
  filter.dynamic_contact_estimation = p.contact.dynamicEstimation;
  filter.contact_foot_indices = p.contact.footIndices;
  filter.contact_release_probability_threshold = p.contact.releaseThreshold;
  filter.contact_point_force_ratio = p.contact.pointForceRatio;
  filter.contact_settle_time = p.contact.settleTime;
  filter.contact_settle_noise = p.contact.settleNoise;

  filter.height_source = p.height.source;
  filter.height_kinematic_weight = p.height.kinematicWeight;
  filter.height_ground_z = p.height.groundZ;
  filter.height_anchor_update_threshold = p.height.anchorUpdateThreshold;
  filter.height_hold_through_flight = p.height.holdThroughFlight;

  filter.lpf_gyro_cutoff = p.lpf.gyroCutoff;
  filter.lpf_gyro_accel_cutoff = p.lpf.gyroAccelCutoff;
  filter.lpf_lin_accel_cutoff = p.lpf.linAccelCutoff;
  filter.lpf_dqJ_cutoff = p.lpf.dqJCutoff;
  filter.lpf_ddqJ_cutoff = p.lpf.ddqJCutoff;
  filter.lpf_tauJ_cutoff = p.lpf.tauJCutoff;
  return true;
}

}  // namespace legged_robot_mpc_controller

