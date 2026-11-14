#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__LINEAR_KF_FLOATING_BASE_ESTIMATOR_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__LINEAR_KF_FLOATING_BASE_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimator_interface.hpp"

namespace legged_robot_mpc_controller::state_estimation
{

/**
 * Two-stage state estimator of MIT Cheetah 3 (Bledt et al., IROS 2018, Sec. III-H),
 * following the OCS2 port in qiayuanl/legged_control.
 *
 * STAGE 1 - attitude. Taken directly from the IMU (setBaseOrientation()). The
 * Cheetah paper instead runs a Mahony complementary filter on raw gyro and
 * accelerometer, de-drifting roll and pitch against gravity with gain
 * kappa_ref = 0.1. Both are "orientation from the IMU alone"; using the sensor's
 * own fused quaternion is what legged_control does and what this port does,
 * since pelvis_imu already publishes one. Either way YAW IS NEVER CORRECTED and
 * accumulates without an exteroceptive source - the paper says so explicitly.
 *
 * STAGE 2 - a conventional linear Kalman filter. Because stage 1 has already
 * fixed attitude, the remaining process is linear time invariant:
 *
 *     p_dot = v
 *     v_dot = R(q) * a_imu + g
 *     d_i_dot = 0                      (a planted foot does not move)
 *
 * That linearity is the whole point of the two-stage split - it is why a plain
 * KF suffices where the InEKF needs a Lie-group formulation, and it is why the
 * paper can claim the filter "will never diverge in finite time".
 *
 * State (6 + 3N, N = number of CONTACT POINTS):
 *     [ base position (3), base velocity (3), contact point positions (3N) ]
 * On the G1 that is 2 feet but N = 4 contact points (heel and toe per foot), so
 * the state is 18-dimensional.
 *
 * Measurements (7N rows), all from forward kinematics of the contact frames:
 *     relative foot position (3N), relative foot velocity (3N), foot height (N)
 * matching the residuals e_p,i / e_v,i / e_h,i of the paper. A foot in SWING has
 * its process and measurement noise multiplied by lkf_swing_noise_multiplier so
 * the filter effectively ignores it.
 *
 * ROLLING WHEELS (lkf_contact_model "rolling_wheel"). A tire's contact point moves
 * with the robot, so it is no landmark; what stays at rest without slip is the tread
 * MATERIAL at the contact. Each wheel then gives
 *     base velocity = -(velocity of that material point relative to the base)   (3 rows)
 *     base height   = ground - (contact point relative to the base).z            (1 row)
 * with the contact at the lowest point of the tire circle, and the state is the base
 * position and velocity only (6). Wheel spin enters through the wheel joint velocity.
 * Hosseini, Rodriguez and Behnke (IRC 2022) adapt legged filters to wheels the same way.
 */
class LinearKfFloatingBaseEstimator : public FloatingBaseEstimatorInterface
{
public:
  explicit LinearKfFloatingBaseEstimator(const EstimatorSettings & settings);

  void initialize(
    const Eigen::Vector3d & base_position,
    const Eigen::Quaterniond & base_orientation,
    const std::vector<double> & joint_positions) override;
  void initializeFromStance(
    const Eigen::Quaterniond & imu_orientation_world,
    const std::vector<double> & joint_positions) override;

  FloatingBaseEstimate update(
    const Eigen::Vector3d & imu_angular_velocity_body,
    const Eigen::Vector3d & imu_linear_acceleration_body,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities,
    const std::vector<double> & joint_torques) override;

  FloatingBaseEstimate update(
    const Eigen::Vector3d & imu_angular_velocity_body,
    const Eigen::Vector3d & imu_linear_acceleration_body,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities,
    const std::vector<double> & joint_torques,
    const std::vector<bool> & foot_contacts) override;

  void reset() override { initialized_ = false; }
  bool initialized() const override { return initialized_; }
  int numEstimatorJoints() const override { return num_estimator_joints_; }

  //! This filter does not estimate IMU biases; the state has no bias block.
  //! Reported as zero rather than left undefined so the diagnostics columns stay
  //! comparable against the InEKF, where they are meaningful.
  Eigen::Vector3d estimatedGyroscopeBias() const override { return Eigen::Vector3d::Zero(); }
  Eigen::Vector3d estimatedAccelerometerBias() const override { return Eigen::Vector3d::Zero(); }

  const EstimatorDiagnostics & diagnostics() const override { return diagnostics_; }
  const std::vector<std::string> & contactFrames() const override { return settings_.contact_frames; }

  void setBaseOrientation(const Eigen::Quaterniond & q) override { base_orientation_ = imuToBaseOrientation(q); }

private:
  FloatingBaseEstimate updateImpl(
    const Eigen::Vector3d & imu_angular_velocity_body,
    const Eigen::Vector3d & imu_linear_acceleration_body,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities,
    const std::vector<bool> & point_contact_flags);

  FloatingBaseEstimate updateRollingWheels(
    const Eigen::Vector3d & base_angular_velocity,
    const Eigen::Vector3d & base_linear_acceleration,
    const std::vector<bool> & wheel_contact_flags);

  /// Tire contact point relative to the base origin (world-aligned) and the velocity of the tread
  /// material there relative to the base, from the last forwardKinematics call.
  void wheelContact(Eigen::Index i, Eigen::Vector3d & position, Eigen::Vector3d & material_velocity) const;

  /// Forward kinematics with the base at the origin: orientation, joints and the base angular rate only.
  void baseRelativeKinematics(
    const Eigen::Vector3d & base_angular_velocity,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities);

  /// Base attitude from the sensor attitude; with lkf_imu_extrinsics the IMU mounting is removed.
  Eigen::Quaterniond imuToBaseOrientation(const Eigen::Quaterniond & imu_orientation) const;

  /// Expand per-FOOT contact flags to per-CONTACT-POINT flags via
  /// settings_.contact_foot_indices. The G1 has 2 feet and 4 contact points.
  std::vector<bool> expandContactFlags(const std::vector<bool> & foot_flags) const;

  void remapToEstimatorOrder(
    const std::vector<double> & controller_values, Eigen::VectorXd & estimator_vector) const;

  EstimatorSettings settings_;
  EstimatorDiagnostics diagnostics_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::FrameIndex> contact_frame_ids_;
  std::vector<int> controller_to_estimator_;   //!< joint index remap
  int num_estimator_joints_{0};

  Eigen::Index num_contacts_{0};   //!< contact POINTS (4 on the G1)
  Eigen::Index dim_contacts_{0};   //!< 3 * num_contacts_
  Eigen::Index num_state_{0};      //!< 6 + dim_contacts_
  Eigen::Index num_observe_{0};    //!< 2 * dim_contacts_ + num_contacts_

  Eigen::MatrixXd a_, b_, c_, q_, p_, r_;
  Eigen::VectorXd x_hat_, ps_, vs_, feet_heights_;

  Eigen::Quaterniond base_orientation_{Eigen::Quaterniond::Identity()};
  bool initialized_{false};

  bool rolling_wheels_{false};
  bool imu_extrinsics_{false};
  Eigen::Quaterniond base_to_imu_{Eigen::Quaterniond::Identity()};   //!< IMU attitude in the base frame
  Eigen::Vector3d imu_position_base_{Eigen::Vector3d::Zero()};       //!< IMU position in the base frame
};

}  // namespace legged_robot_mpc_controller::state_estimation

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__LINEAR_KF_FLOATING_BASE_ESTIMATOR_HPP_
