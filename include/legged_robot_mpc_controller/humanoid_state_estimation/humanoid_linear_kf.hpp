// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_LINEAR_KF_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_LINEAR_KF_HPP_

#include <memory>

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_state_estimator.hpp"

namespace legged_robot_mpc_controller
{

/**
 * Two-stage linear Kalman filter floating-base estimator (Bledt et al. 2018).
 * Stage 1 takes attitude straight from the IMU; stage 2 estimates base position
 * and velocity with the stance feet as position measurements. Kept so it can be
 * compared against HumanoidInEkf on the same logs.
 *
 * Declares only this filter's parameters, under humanoid_linear_kf.* on the
 * controller's node, from its humanoid_linear_kf block in config/g1/ros2_controllers.yaml.
 */
class HumanoidLinearKf : public HumanoidStateEstimator
{
public:
  HumanoidLinearKf();
  ~HumanoidLinearKf() override;

protected:
  state_estimation::EstimatorType estimatorType() const override
  {
    return state_estimation::EstimatorType::LinearKF;
  }
  bool declareParameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) override;
  bool readParameters(
    EstimatorCommonSettings & common, state_estimation::EstimatorSettings & filter) override;

private:
  // The generated parameter types are only visible in the .cpp.
  struct Params;
  std::unique_ptr<Params> params_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_LINEAR_KF_HPP_
