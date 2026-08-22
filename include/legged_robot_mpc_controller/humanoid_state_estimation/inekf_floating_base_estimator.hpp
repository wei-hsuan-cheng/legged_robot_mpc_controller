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

    // Initial InEKF covariance: how much the pose passed to initialize() is
    // trusted. The library default is P = I(15) - 1 rad of attitude uncertainty,
    // 1 m/s of velocity, 1 m of position - even when seeded from ground truth,
    // so the first contact correction (millimetre noise) yanks the state. The
    // resulting transient is easily mistaken for slow bias convergence.
    double initial_attitude_noise{1.0e-2};            //!< [rad]
    double initial_velocity_noise{1.0e-2};            //!< [m/s]
    double initial_position_noise{1.0e-2};            //!< [m]
    double initial_gyroscope_bias_noise{1.0e-2};      //!< [rad/s]
    double initial_accelerometer_bias_noise{1.0e-1};  //!< [m/s^2]

    //! Estimate the IMU biases. The bias is observable only through its
    //! cross-covariance with the attitude/velocity/position block (the kinematic
    //! measurement Jacobian has no bias columns), so the initial bias covariance
    //! above must be non-zero for this to have any effect.
    bool estimate_imu_bias{true};

    // Torque-based contact estimator (logistic regression), one entry per contact.
    std::vector<double> contact_beta0;
    std::vector<double> contact_beta1;
    double contact_force_covariance_alpha{100.0};
    double contact_probability_threshold{0.5};
    bool dynamic_contact_estimation{false};
    std::string contact_source{"torque"};             //!< "torque" or externally supplied "scheduled"
    std::vector<int64_t> contact_foot_indices;        //!< foot index for each point-contact frame

    // Base-height conditioning. The centroidal MPC references swing/stance foot
    // targets to an ABSOLUTE ground plane (terrainHeight is forced to 0 in
    // SwitchedModelReferenceManager::adaptToCurrentGroundHeight), so any base-height
    // error shifts every foot height relative to that plane 1:1 - a few centimetres
    // is a large fraction of the 0.08 m swing height and destabilizes contact timing.
    // Fusing the InEKF height with the height implied by the stance-foot kinematics
    // (feet on z = 0) keeps the observation consistent with what the MPC assumes.
    // "inekf"     = raw filter height.
    // "kinematic" = stance-foot height against a flat plane at height_ground_z.
    // "blend"     = complementary filter between the two (flat ground only).
    // "anchored"  = per-contact, time-varying ground height: each contact records the
    //               estimated world height where it touched down and keeps it through
    //               stance, so the kinematic term enforces consistency *within* a
    //               stance phase without fighting a real terrain change. Valid on
    //               stairs and slopes, and needs no terrain map.
    std::string height_source{"inekf"};
    double height_kinematic_weight{0.0};  //!< blend weight on the kinematic height, [0, 1]
    double height_ground_z{0.0};          //!< flat-plane height [m] for "kinematic"/"blend"
    //! "anchored" only: a touchdown re-anchors a contact just when the measured landing
    //! height differs from its current anchor by more than this [m]. It must exceed the
    //! filter's height noise (~0.03 m) so flat ground keeps its exact, noise-free anchor,
    //! and stay well below a stair riser (0.17 m) so real steps are still captured.
    double height_anchor_update_threshold{0.05};

    // Low-pass filter cutoff frequencies [Hz].
    double lpf_gyro_cutoff{250.0};
    double lpf_gyro_accel_cutoff{250.0};
    double lpf_lin_accel_cutoff{250.0};
    double lpf_dqJ_cutoff{10.0};
    double lpf_ddqJ_cutoff{5.0};
    double lpf_tauJ_cutoff{10.0};
  };

  /**
   * Per-update view of the filter's internals, for offline diagnosis.
   *
   * The estimate alone cannot distinguish a filter that is confident and right
   * from one that is confident and wrong. These are the quantities that can:
   * the covariance it reports for itself, the innovation it saw relative to the
   * covariance it predicted for that innovation (NIS), the contact bookkeeping
   * that drives both, and the two competing height sources that the reported
   * height is blended from.
   *
   * All per-contact vectors are indexed by contact frame (settings.contactFrames
   * order) and are sized once at construction, so filling them allocates nothing.
   */
  struct Diagnostics
  {
    bool valid{false};

    // --- last InEKF measurement update -------------------------------------
    bool correction_applied{false};   //!< a correction actually fired this tick
    int correction_dim{0};            //!< stacked measurement dimension (3 per corrected contact)
    double nis{0.0};                  //!< normalized innovation squared, r' S^-1 r
    //! NIS divided by its degrees of freedom. A consistent filter sits near 1;
    //! persistently below 1 means the measurement noise is overstated, above
    //! means the filter is over-confident (the signature of double-counting
    //! rigidly-linked heel/toe landmarks as independent measurements).
    double nis_per_dof{0.0};

    //! Per-contact innovation r and its predicted std sqrt(diag(S)), world frame.
    //! Zero for contacts that did not take part in this correction.
    std::vector<Eigen::Vector3d> contact_innovation;
    std::vector<Eigen::Vector3d> contact_innovation_std;
    std::vector<bool> contact_corrected;   //!< contact contributed to this correction
    std::vector<bool> contact_added;       //!< contact was augmented this tick (touchdown)
    std::vector<bool> contact_removed;     //!< contact was marginalized this tick (liftoff)
    std::vector<bool> contact_in_stance;   //!< scheduled/detected contact flag fed to the filter

    //! World position the filter holds for each augmented contact landmark. A
    //! stance-phase drift here is the "foot stretch" a position-only correction
    //! has to absorb into the base pose when the real foot rolls.
    std::vector<Eigen::Vector3d> contact_landmark_position;
    std::vector<bool> contact_landmark_valid;

    int num_augmented_contacts{0};

    // --- covariance ---------------------------------------------------------
    Eigen::Vector3d attitude_std{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_std{Eigen::Vector3d::Zero()};
    Eigen::Vector3d position_std{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyroscope_bias_std{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accelerometer_bias_std{Eigen::Vector3d::Zero()};

    // --- bias ---------------------------------------------------------------
    Eigen::Vector3d gyroscope_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accelerometer_bias{Eigen::Vector3d::Zero()};
    bool estimating_bias{false};

    // --- height conditioning ------------------------------------------------
    //! The filter's own base height, BEFORE the kinematic blend. The blend is
    //! applied outside the filter, so this keeps drifting uncorrected even while
    //! the reported height looks good - and the touchdown anchors are computed
    //! from it, so its drift compounds into them.
    double inekf_height{0.0};
    double kinematic_height{0.0};
    bool kinematic_height_valid{false};
    double reported_height{0.0};      //!< what the MPC actually receives
    std::vector<double> contact_ground_anchor;
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

  /// Filter internals from the most recent update(); see Diagnostics.
  const Diagnostics & diagnostics() const { return diagnostics_; }

  /// Contact frame names, matching the per-contact ordering used in Diagnostics.
  const std::vector<std::string> & contactFrames() const { return settings_.contact_frames; }

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
