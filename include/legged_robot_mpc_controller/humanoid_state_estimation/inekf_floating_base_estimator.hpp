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

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimator_interface.hpp"

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
 * zero-fills any URDF joints the controller does not actuate.
 */
class InekfFloatingBaseEstimator : public FloatingBaseEstimatorInterface
{
public:
  //! Kept so existing call sites (state_estimation::InekfFloatingBaseEstimator::Settings)
  //! keep compiling now that the types are shared with the linear KF.
  using Settings = EstimatorSettings;
  using Diagnostics = EstimatorDiagnostics;



  explicit InekfFloatingBaseEstimator(const Settings& settings);

  /// Bootstrap the filter from a known base pose (e.g. from MuJoCo GT at activation).
  /// joint_positions are in the controller's jointNames order.
  void initialize(
    const Eigen::Vector3d& base_position,
    const Eigen::Quaterniond& base_orientation,
    const std::vector<double>& joint_positions) override;

  /// One estimator step. IMU quantities are body-local raw measurements; joint
  /// vectors are in the controller's jointNames order.
  FloatingBaseEstimate update(
    const Eigen::Vector3d& imu_gyro_body,
    const Eigen::Vector3d& imu_linear_acceleration_body,
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_efforts) override;

  /// One estimator step using externally supplied per-foot contact states.
  FloatingBaseEstimate update(
    const Eigen::Vector3d& imu_gyro_body,
    const Eigen::Vector3d& imu_linear_acceleration_body,
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_efforts,
    const std::vector<bool>& foot_contacts) override;

  /// Forget the initialization so the next initialize() re-seeds the filter (e.g. on re-activation).
  void reset() override { initialized_ = false; }

  bool initialized() const override { return initialized_; }

  /// Number of actuated joints in the estimator (URDF) model.
  int numEstimatorJoints() const override { return num_estimator_joints_; }

  /// InEKF bias estimates (for diagnostics); valid after the first update().
  Eigen::Vector3d estimatedAccelerometerBias() const override
  {
    return estimator_ ? estimator_->getIMULinearAccelerationBiasEstimate() : Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d estimatedGyroscopeBias() const override
  {
    return estimator_ ? estimator_->getIMUGyroBiasEstimate() : Eigen::Vector3d::Zero();
  }

  /// Filter internals from the most recent update(); see Diagnostics.
  const EstimatorDiagnostics & diagnostics() const override { return diagnostics_; }

  /// Contact frame names, matching the per-contact ordering used in Diagnostics.
  const std::vector<std::string> & contactFrames() const override { return settings_.contact_frames; }

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

  /// Contact heights relative to the base (base at the origin, current orientation and
  /// joint angles), i.e. [^B p_Ci]_z for every configured contact frame.
  void contactHeightsRelativeToBase(
    const Eigen::Quaterniond& base_orientation,
    std::vector<double>& relative_heights_out);

  /// True when the given contact frame is in stance (all contacts count as stance when
  /// no external contact vector is supplied).
  bool isContactInStance(size_t contact, const std::vector<bool>* foot_contacts) const;

  /// Base height that places the stance contacts on their ground reference: a flat plane
  /// at height_ground_z for "kinematic"/"blend", or the per-contact touchdown anchor for
  /// "anchored". Returns false when no contact is active (nothing to anchor to), in which
  /// case the InEKF height is kept. Also refreshes the touchdown anchors.
  bool kinematicBaseHeight(
    const Eigen::Quaterniond& base_orientation,
    const std::vector<bool>* foot_contacts,
    double inekf_base_height,
    double& height_out);

  /// Size every per-contact Diagnostics vector once, so update() never allocates.
  void allocateDiagnostics();

  /// Copy the filter internals for this tick into diagnostics_. Called from
  /// updateImpl() after the InEKF has run and the height terms are known.
  void collectDiagnostics(const std::vector<bool>* foot_contacts);

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

  // Per-contact ground reference for height_source == "anchored": the estimated world
  // height at which each contact last touched down, held for the duration of stance.
  std::vector<double> contact_ground_anchor_;
  std::vector<bool> contact_was_in_stance_;
  std::vector<double> contact_relative_heights_;

  Diagnostics diagnostics_;

  bool initialized_{false};
};



}  // namespace legged_robot_mpc_controller::state_estimation

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__INEKF_FLOATING_BASE_ESTIMATOR_HPP_
