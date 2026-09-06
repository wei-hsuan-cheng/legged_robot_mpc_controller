#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__FLOATING_BASE_ESTIMATOR_INTERFACE_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__FLOATING_BASE_ESTIMATOR_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimate.hpp"

namespace legged_robot_mpc_controller::state_estimation
{

/**
 * Which floating-base estimator the controller should run.
 *
 * Both fuse the same signals - IMU plus forward kinematics of the contact feet -
 * and differ in where they linearise.
 *
 *  InEkf     Contact-aided right-invariant EKF (Hartley et al., 2018). The state
 *            lives on SE_{N+2}(3) and the process model is group affine, so the
 *            right-invariant error obeys a LOG-LINEAR autonomous ODE.
 *            Convergence therefore does not depend on the current estimate, and
 *            the observability structure is exact rather than an artefact of
 *            linearisation, so the filter cannot spuriously "observe" yaw.
 *
 *  LinearKf  Two-stage design of MIT Cheetah 3 (Bledt et al., IROS 2018,
 *            Sec. III-H), as ported in qiayuanl/legged_control. Stage 1 takes
 *            attitude from the IMU; stage 2 is a CONVENTIONAL linear Kalman
 *            filter over base position, base velocity and the contact-point
 *            positions. Because attitude is treated as known, the remaining
 *            process is linear time-invariant, which is what lets a plain KF be
 *            used - the paper notes this "simplifies analysis and tuning and
 *            guarantees the filter equations will never diverge in finite time".
 *
 * Neither observes yaw or absolute xy. The InEKF knows that structurally; the
 * two-stage filter simply inherits whatever yaw its attitude source reports.
 */
enum class EstimatorType
{
  InEkf,
  LinearKf,
};

/// Parse the `stateEstimator.estimatorType` string. Throws on an unknown value.
EstimatorType estimatorTypeFromString(const std::string & name);

/// Human-readable name, for logs and errors.
const char * toString(EstimatorType type);

struct EstimatorSettings
{
  /// Physical feet, derived from the contact->foot map unless set explicitly.
  int numFeet() const;

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

  //! Number of physical FEET, which is NOT the number of contact frames.
  //!
  //! The G1 has 2 feet but 4 contact points - heel and toe per foot - so
  //! contact_frames has 4 entries while the gait schedule produces 2 contact
  //! flags. contact_foot_indices maps one to the other ([0,0,1,1] here).
  //! 0 means "derive it as max(contact_foot_indices) + 1", which is correct
  //! whenever the mapping is dense; set it explicitly only to assert a value.
  int num_feet{0};

  // ---- linear KF (estimatorType: linearKF), MIT Cheetah 3 two-stage design ----
  //! Contact-point sphere radius, added to the measured foot height [m].
  double lkf_foot_radius{0.0};
  //! Process noise on the IMU-integrated base position / velocity.
  double lkf_imu_process_noise_position{0.02};
  double lkf_imu_process_noise_velocity{0.02};
  //! Process noise on a stance foot's world position (i.e. allowed slip).
  double lkf_foot_process_noise_position{0.002};
  //! Measurement noise on the forward-kinematic foot position / velocity.
  double lkf_foot_sensor_noise_position{0.005};
  double lkf_foot_sensor_noise_velocity{0.1};
  //! Measurement noise on the assumed contact height.
  double lkf_foot_height_sensor_noise{0.01};
  //! Multiplier applied to a SWING foot\'s process and measurement noise, which
  //! is how the filter ignores a foot that is not on the ground. Cheetah 3 uses
  //! 100; the paper describes it as raising the covariance "to a high value".
  double lkf_swing_noise_multiplier{100.0};

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

struct EstimatorDiagnostics
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

/**
 * The surface the controller needs from a floating-base estimator.
 *
 * Deliberately small: everything the control loop touches, and nothing about how
 * the estimate is produced. Diagnostics is InEKF-shaped (NIS, per-contact
 * innovation, landmarks); an estimator without those concepts leaves them unset,
 * and DiagnosticsCsvLogger writes unset columns empty - which numpy/pandas read
 * back as NaN and is therefore distinguishable from a real zero.
 */
class FloatingBaseEstimatorInterface
{
public:
  virtual ~FloatingBaseEstimatorInterface() = default;

  virtual void initialize(
    const Eigen::Vector3d & base_position,
    const Eigen::Quaterniond & base_orientation,
    const std::vector<double> & joint_positions) = 0;

  virtual FloatingBaseEstimate update(
    const Eigen::Vector3d & imu_angular_velocity_body,
    const Eigen::Vector3d & imu_linear_acceleration_body,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities,
    const std::vector<double> & joint_torques) = 0;

  virtual FloatingBaseEstimate update(
    const Eigen::Vector3d & imu_angular_velocity_body,
    const Eigen::Vector3d & imu_linear_acceleration_body,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities,
    const std::vector<double> & joint_torques,
    const std::vector<bool> & foot_contacts) = 0;

  virtual void reset() = 0;
  virtual bool initialized() const = 0;
  virtual int numEstimatorJoints() const = 0;
  virtual Eigen::Vector3d estimatedGyroscopeBias() const = 0;
  virtual Eigen::Vector3d estimatedAccelerometerBias() const = 0;
  virtual const EstimatorDiagnostics & diagnostics() const = 0;
  virtual const std::vector<std::string> & contactFrames() const = 0;

  /**
   * Attitude source for estimators that do not estimate it themselves.
   *
   * The InEKF ignores this: attitude is part of its state. The two-stage linear
   * KF needs it, because its stage 2 assumes orientation is already known - that
   * assumption is exactly what makes the remaining process linear.
   */
  virtual void setBaseOrientation(const Eigen::Quaterniond &) {}
};

/**
 * Construct the configured floating-base estimator.
 *
 * @throws std::invalid_argument if the requested type is unknown.
 */
std::unique_ptr<FloatingBaseEstimatorInterface> makeFloatingBaseEstimator(
  EstimatorType type, const EstimatorSettings & settings);

}  // namespace legged_robot_mpc_controller::state_estimation

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__FLOATING_BASE_ESTIMATOR_INTERFACE_HPP_
