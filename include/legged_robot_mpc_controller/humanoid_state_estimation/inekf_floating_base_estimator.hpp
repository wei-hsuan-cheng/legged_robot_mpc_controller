#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__INEKF_FLOATING_BASE_ESTIMATOR_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__INEKF_FLOATING_BASE_ESTIMATOR_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <legged_state_estimator/legged_state_estimator.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimate.hpp"

namespace legged_robot_mpc_controller::state_estimation
{

/**
 * Contact-aided Invariant EKF floating-base estimator (Hartley et al. 2018),
 * wrapping legged_state_estimator::LeggedStateEstimator.
 *
 * It fuses the pelvis IMU (raw gyro + accelerometer) with leg forward kinematics
 * and torque-based contact estimation to produce the base pose and velocity,
 * removing the dependence on the MuJoCo ground-truth body frame in the control
 * loop. The InEKF estimates orientation jointly (no separate AHRS needed).
 *
 * The controller provides joint states in its own `robot.jointNames` order; this
 * wrapper remaps them into the estimator URDF's Pinocchio joint order and
 * zero-fills any URDF joints the controller does not actuate (e.g. wrists).
 */
class InekfFloatingBaseEstimator
{
public:
  struct Settings
  {
    std::string urdf_path;
    std::string imu_frame;                            //!< URDF frame of the IMU (e.g. "imu_in_pelvis")
    std::vector<std::string> contact_frames;          //!< URDF contact frames (e.g. foot_l/r_contact)
    std::vector<std::string> controller_joint_names;  //!< order the controller provides joint values in
    double sampling_time{0.001};                      //!< estimator dt (1 / controller update rate)

    // InEKF process/measurement noise std-devs (isotropic).
    double gyroscope_noise{0.01};
    double accelerometer_noise{0.1};
    double gyroscope_bias_noise{1.0e-5};
    double accelerometer_bias_noise{1.0e-4};
    double contact_noise{0.1};
    double contact_position_noise{0.01};
    double contact_rotation_noise{0.01};

    // Torque-based contact estimator (logistic regression), one entry per contact.
    std::vector<double> contact_beta0;
    std::vector<double> contact_beta1;
    double contact_force_covariance_alpha{100.0};
    double contact_probability_threshold{0.5};
    bool dynamic_contact_estimation{false};
    std::string contact_source{"torque"};             //!< "torque" or externally supplied "scheduled"
    std::vector<int64_t> contact_foot_indices;        //!< foot index for each point-contact frame

    // Low-pass filter cutoff frequencies [Hz].
    double lpf_gyro_cutoff{250.0};
    double lpf_gyro_accel_cutoff{250.0};
    double lpf_lin_accel_cutoff{250.0};
    double lpf_dqJ_cutoff{10.0};
    double lpf_ddqJ_cutoff{5.0};
    double lpf_tauJ_cutoff{10.0};
  };

  explicit InekfFloatingBaseEstimator(const Settings& settings);

  /// Bootstrap the filter from a known base pose (e.g. from MuJoCo GT at activation).
  /// joint_positions are in the controller's jointNames order.
  void initialize(
    const Eigen::Vector3d& base_position,
    const Eigen::Quaterniond& base_orientation,
    const std::vector<double>& joint_positions);

  /// One estimator step. IMU quantities are body-local raw measurements; joint
  /// vectors are in the controller's jointNames order.
  FloatingBaseEstimate update(
    const Eigen::Vector3d& imu_gyro_body,
    const Eigen::Vector3d& imu_linear_acceleration_body,
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_efforts);

  /// One estimator step using externally supplied per-foot contact states.
  FloatingBaseEstimate update(
    const Eigen::Vector3d& imu_gyro_body,
    const Eigen::Vector3d& imu_linear_acceleration_body,
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_efforts,
    const std::vector<bool>& foot_contacts);

  /// Forget the initialization so the next initialize() re-seeds the filter (e.g. on re-activation).
  void reset() { initialized_ = false; }

  bool initialized() const { return initialized_; }

  /// Number of actuated joints in the estimator (URDF) model.
  int numEstimatorJoints() const { return num_estimator_joints_; }

  /// InEKF bias estimates (for diagnostics); valid after the first update().
  Eigen::Vector3d estimatedAccelerometerBias() const
  {
    return estimator_ ? estimator_->getIMULinearAccelerationBiasEstimate() : Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d estimatedGyroscopeBias() const
  {
    return estimator_ ? estimator_->getIMUGyroBiasEstimate() : Eigen::Vector3d::Zero();
  }

private:
  /// Fill an estimator-order joint vector from a controller-order one (unmapped joints stay 0).
  void remapToEstimatorOrder(const std::vector<double>& controller_values, Eigen::VectorXd& estimator_vector) const;

  /// World-frame z of each contact frame at the given base pose + joints, used to
  /// seed the InEKF contact landmarks at their true (per-foot) heights so the filter
  /// is consistent on flat ground and on stairs.
  std::vector<double> contactGroundHeights(
    const Eigen::Vector3d& base_position,
    const Eigen::Quaterniond& base_orientation,
    const Eigen::VectorXd& estimator_joint_positions);

  FloatingBaseEstimate updateImpl(
    const Eigen::Vector3d& imu_gyro_body,
    const Eigen::Vector3d& imu_linear_acceleration_body,
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_efforts,
    const std::vector<bool>* foot_contacts);

  Settings settings_;
  std::unique_ptr<legged_state_estimator::LeggedStateEstimator> estimator_;

  // Pinocchio model/data mirror of the estimator URDF, for joint remap + contact FK.
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<pinocchio::FrameIndex> contact_frame_ids_;

  // Fixed transform of the IMU frame in the floating-base (pelvis) frame. The InEKF
  // estimates the IMU frame pose; the controller wants the pelvis, so init converts
  // pelvis -> imu and update() converts imu -> pelvis with this transform.
  Eigen::Vector3d base_to_imu_translation_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d base_to_imu_rotation_{Eigen::Matrix3d::Identity()};

  int num_estimator_joints_{0};
  /// controller_to_estimator_qj_index_[i] = estimator qJ index of controller joint i, or -1.
  std::vector<int> controller_to_estimator_qj_index_;

  Eigen::VectorXd qj_, dqj_, tauj_;
  bool initialized_{false};
};

}  // namespace legged_robot_mpc_controller::state_estimation

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__INEKF_FLOATING_BASE_ESTIMATOR_HPP_
