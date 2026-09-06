#include "legged_robot_mpc_controller/humanoid_state_estimation/inekf_floating_base_estimator.hpp"

#include "legged_robot_mpc_controller/humanoid_state_estimation/linear_kf_floating_base_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace legged_robot_mpc_controller::state_estimation
{

namespace
{

legged_state_estimator::LeggedStateEstimatorSettings toLibrarySettings(
  const InekfFloatingBaseEstimator::Settings& s)
{
  legged_state_estimator::LeggedStateEstimatorSettings out;
  out.urdf_path = s.urdf_path;
  out.imu_frame = s.imu_frame;
  out.contact_frames = s.contact_frames;

  out.contact_estimator_settings.beta0 = s.contact_beta0;
  out.contact_estimator_settings.beta1 = s.contact_beta1;
  out.contact_estimator_settings.contact_force_covariance_alpha = s.contact_force_covariance_alpha;
  out.contact_estimator_settings.contact_probability_threshold = s.contact_probability_threshold;

  out.inekf_noise_params.setGyroscopeNoise(s.gyroscope_noise);
  out.inekf_noise_params.setAccelerometerNoise(s.accelerometer_noise);
  out.inekf_noise_params.setGyroscopeBiasNoise(s.gyroscope_bias_noise);
  out.inekf_noise_params.setAccelerometerBiasNoise(s.accelerometer_bias_noise);
  out.inekf_noise_params.setContactNoise(s.contact_noise);

  out.dynamic_contact_estimation = s.dynamic_contact_estimation;
  out.contact_position_noise = s.contact_position_noise;
  out.contact_rotation_noise = s.contact_rotation_noise;
  out.sampling_time = s.sampling_time;

  out.initial_attitude_noise = s.initial_attitude_noise;
  out.initial_velocity_noise = s.initial_velocity_noise;
  out.initial_position_noise = s.initial_position_noise;
  out.initial_gyroscope_bias_noise = s.initial_gyroscope_bias_noise;
  out.initial_accelerometer_bias_noise = s.initial_accelerometer_bias_noise;
  out.estimate_imu_bias = s.estimate_imu_bias;

  out.lpf_gyro_cutoff_frequency = s.lpf_gyro_cutoff;
  out.lpf_gyro_accel_cutoff_frequency = s.lpf_gyro_accel_cutoff;
  out.lpf_lin_accel_cutoff_frequency = s.lpf_lin_accel_cutoff;
  out.lpf_dqJ_cutoff_frequency = s.lpf_dqJ_cutoff;
  out.lpf_ddqJ_cutoff_frequency = s.lpf_ddqJ_cutoff;
  out.lpf_tauJ_cutoff_frequency = s.lpf_tauJ_cutoff;
  return out;
}

}  // namespace

InekfFloatingBaseEstimator::InekfFloatingBaseEstimator(const Settings& settings)
: settings_(settings)
{
  if (settings_.contact_beta0.size() != settings_.contact_frames.size() ||
      settings_.contact_beta1.size() != settings_.contact_frames.size()) {
    throw std::invalid_argument(
      "[InekfFloatingBaseEstimator] contact_beta0/beta1 must have one entry per contact frame");
  }
  if (settings_.height_source != "inekf" && settings_.height_source != "kinematic" &&
      settings_.height_source != "blend" && settings_.height_source != "anchored") {
    throw std::invalid_argument(
      "[InekfFloatingBaseEstimator] height_source must be 'inekf', 'kinematic', 'blend' or 'anchored'");
  }
  if (settings_.contact_source != "torque" && settings_.contact_source != "scheduled") {
    throw std::invalid_argument(
      "[InekfFloatingBaseEstimator] contact_source must be 'torque' or 'scheduled'");
  }
  if (settings_.contact_source == "scheduled" &&
      settings_.contact_foot_indices.size() != settings_.contact_frames.size()) {
    throw std::invalid_argument(
      "[InekfFloatingBaseEstimator] scheduled contact source requires one foot index per contact frame");
  }

  // Build a Pinocchio floating-base model from the same URDF the estimator uses, to
  // remap the controller's joint ordering into the estimator's Pinocchio joint order
  // and to compute the contact-frame heights that seed the InEKF landmarks.
  pinocchio::urdf::buildModel(settings_.urdf_path, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  num_estimator_joints_ = static_cast<int>(model_.nq) - 7;  // free-flyer base occupies 7 config dims
  if (num_estimator_joints_ < 0) {
    throw std::runtime_error("[InekfFloatingBaseEstimator] URDF has no actuated joints");
  }

  controller_to_estimator_qj_index_.assign(settings_.controller_joint_names.size(), -1);
  for (size_t i = 0; i < settings_.controller_joint_names.size(); ++i) {
    const std::string& name = settings_.controller_joint_names[i];
    if (!model_.existJointName(name)) {
      throw std::invalid_argument(
        "[InekfFloatingBaseEstimator] controller joint '" + name + "' not found in estimator URDF");
    }
    const auto joint_id = model_.getJointId(name);
    // Config index of this joint minus the 7 base dims gives its slot in the qJ vector.
    controller_to_estimator_qj_index_[i] = static_cast<int>(model_.idx_qs[joint_id]) - 7;
  }

  contact_frame_ids_.reserve(settings_.contact_frames.size());
  for (const std::string& frame : settings_.contact_frames) {
    if (!model_.existFrame(frame)) {
      throw std::invalid_argument(
        "[InekfFloatingBaseEstimator] contact frame '" + frame + "' not found in estimator URDF");
    }
    contact_frame_ids_.push_back(model_.getFrameId(frame));
  }

  // Fixed IMU-in-base transform: the IMU frame is rigidly attached to the floating
  // base (pelvis) root, so its placement in the base frame is configuration-independent.
  if (!model_.existFrame(settings_.imu_frame)) {
    throw std::invalid_argument(
      "[InekfFloatingBaseEstimator] imu frame '" + settings_.imu_frame + "' not found in estimator URDF");
  }
  {
    const Eigen::VectorXd q_neutral = pinocchio::neutral(model_);  // base at origin, joints 0
    pinocchio::forwardKinematics(model_, data_, q_neutral);
    const auto imu_frame_id = model_.getFrameId(settings_.imu_frame);
    pinocchio::updateFramePlacement(model_, data_, imu_frame_id);
    base_to_imu_translation_ = data_.oMf[imu_frame_id].translation();
    base_to_imu_rotation_ = data_.oMf[imu_frame_id].rotation();
  }

  qj_ = Eigen::VectorXd::Zero(num_estimator_joints_);
  dqj_ = Eigen::VectorXd::Zero(num_estimator_joints_);
  tauj_ = Eigen::VectorXd::Zero(num_estimator_joints_);

  estimator_ = std::make_unique<legged_state_estimator::LeggedStateEstimator>(
    toLibrarySettings(settings_));

  allocateDiagnostics();
}

void InekfFloatingBaseEstimator::allocateDiagnostics()
{
  const size_t n = contact_frame_ids_.size();
  diagnostics_.contact_innovation.assign(n, Eigen::Vector3d::Zero());
  diagnostics_.contact_innovation_std.assign(n, Eigen::Vector3d::Zero());
  diagnostics_.contact_corrected.assign(n, false);
  diagnostics_.contact_added.assign(n, false);
  diagnostics_.contact_removed.assign(n, false);
  diagnostics_.contact_in_stance.assign(n, false);
  diagnostics_.contact_landmark_position.assign(n, Eigen::Vector3d::Zero());
  diagnostics_.contact_landmark_valid.assign(n, false);
  diagnostics_.contact_ground_anchor.assign(n, 0.0);
}

void InekfFloatingBaseEstimator::remapToEstimatorOrder(
  const std::vector<double>& controller_values, Eigen::VectorXd& estimator_vector) const
{
  estimator_vector.setZero();
  for (size_t i = 0; i < controller_to_estimator_qj_index_.size() && i < controller_values.size(); ++i) {
    const int idx = controller_to_estimator_qj_index_[i];
    if (idx >= 0 && idx < estimator_vector.size()) {
      estimator_vector(idx) = controller_values[i];
    }
  }
}

std::vector<double> InekfFloatingBaseEstimator::contactGroundHeights(
  const Eigen::Vector3d& base_position,
  const Eigen::Quaterniond& base_orientation,
  const Eigen::VectorXd& estimator_joint_positions)
{
  Eigen::VectorXd q(model_.nq);
  q.head<3>() = base_position;
  q.segment<4>(3) = Eigen::Vector4d(
    base_orientation.x(), base_orientation.y(), base_orientation.z(), base_orientation.w());
  q.tail(num_estimator_joints_) = estimator_joint_positions;

  pinocchio::forwardKinematics(model_, data_, q);
  std::vector<double> heights;
  heights.reserve(contact_frame_ids_.size());
  for (const auto frame_id : contact_frame_ids_) {
    pinocchio::updateFramePlacement(model_, data_, frame_id);
    heights.push_back(data_.oMf[frame_id].translation().z());
  }
  return heights;
}

void InekfFloatingBaseEstimator::initialize(
  const Eigen::Vector3d& base_position,
  const Eigen::Quaterniond& base_orientation,
  const std::vector<double>& joint_positions)
{
  remapToEstimatorOrder(joint_positions, qj_);

  // The InEKF tracks the IMU frame; convert the incoming pelvis pose to the IMU frame.
  const Eigen::Matrix3d base_rotation = base_orientation.toRotationMatrix();
  const Eigen::Matrix3d imu_rotation = base_rotation * base_to_imu_rotation_;
  const Eigen::Vector3d imu_position = base_position + base_rotation * base_to_imu_translation_;
  const Eigen::Quaterniond imu_quat(imu_rotation);
  const Eigen::Vector4d quat_xyzw(imu_quat.x(), imu_quat.y(), imu_quat.z(), imu_quat.w());

  // Seed each contact landmark at its true world height (from FK at the seed pose),
  // so the filter is consistent with the actual foot geometry / terrain start heights
  // instead of assuming a flat ground at z = 0.
  const std::vector<double> ground_heights = contactGroundHeights(base_position, base_orientation, qj_);
  // Seed the per-contact ground anchors ("anchored" height source) with the same
  // heights, and mark every contact as fresh so the first stance re-anchors cleanly.
  contact_ground_anchor_ = ground_heights;
  contact_was_in_stance_.assign(contact_frame_ids_.size(), false);
  estimator_->init(imu_position, quat_xyzw, qj_, ground_heights);
  initialized_ = true;
}

FloatingBaseEstimate InekfFloatingBaseEstimator::update(
  const Eigen::Vector3d& imu_gyro_body,
  const Eigen::Vector3d& imu_linear_acceleration_body,
  const std::vector<double>& joint_positions,
  const std::vector<double>& joint_velocities,
  const std::vector<double>& joint_efforts)
{
  return updateImpl(
    imu_gyro_body, imu_linear_acceleration_body, joint_positions,
    joint_velocities, joint_efforts, nullptr);
}

FloatingBaseEstimate InekfFloatingBaseEstimator::update(
  const Eigen::Vector3d& imu_gyro_body,
  const Eigen::Vector3d& imu_linear_acceleration_body,
  const std::vector<double>& joint_positions,
  const std::vector<double>& joint_velocities,
  const std::vector<double>& joint_efforts,
  const std::vector<bool>& foot_contacts)
{
  return updateImpl(
    imu_gyro_body, imu_linear_acceleration_body, joint_positions,
    joint_velocities, joint_efforts, &foot_contacts);
}

void InekfFloatingBaseEstimator::contactHeightsRelativeToBase(
  const Eigen::Quaterniond& base_orientation,
  std::vector<double>& relative_heights_out)
{
  // FK with the base at the world origin: oMf[contact].z is then the contact height
  // relative to the base, expressed in a world-aligned frame.
  Eigen::VectorXd q(model_.nq);
  q.head<3>().setZero();
  q.segment<4>(3) = Eigen::Vector4d(
    base_orientation.x(), base_orientation.y(), base_orientation.z(), base_orientation.w());
  q.tail(num_estimator_joints_) = qj_;

  pinocchio::forwardKinematics(model_, data_, q);
  relative_heights_out.resize(contact_frame_ids_.size());
  for (size_t contact = 0; contact < contact_frame_ids_.size(); ++contact) {
    pinocchio::updateFramePlacement(model_, data_, contact_frame_ids_[contact]);
    relative_heights_out[contact] = data_.oMf[contact_frame_ids_[contact]].translation().z();
  }
}

bool InekfFloatingBaseEstimator::isContactInStance(
  size_t contact, const std::vector<bool>* foot_contacts) const
{
  if (foot_contacts == nullptr) {
    return true;  // no contact information: treat every configured contact as support
  }
  const int64_t foot = settings_.contact_foot_indices[contact];
  if (foot < 0 || static_cast<size_t>(foot) >= foot_contacts->size()) {
    return false;
  }
  return (*foot_contacts)[static_cast<size_t>(foot)];
}

bool InekfFloatingBaseEstimator::kinematicBaseHeight(
  const Eigen::Quaterniond& base_orientation,
  const std::vector<bool>* foot_contacts,
  double inekf_base_height,
  double& height_out)
{
  contactHeightsRelativeToBase(base_orientation, contact_relative_heights_);

  const bool anchored = (settings_.height_source == "anchored");
  if (contact_ground_anchor_.size() != contact_frame_ids_.size()) {
    contact_ground_anchor_.assign(contact_frame_ids_.size(), settings_.height_ground_z);
    contact_was_in_stance_.assign(contact_frame_ids_.size(), false);
  }

  double sum = 0.0;
  int count = 0;
  for (size_t contact = 0; contact < contact_frame_ids_.size(); ++contact) {
    const bool in_stance = isContactInStance(contact, foot_contacts);

    if (anchored && in_stance && !contact_was_in_stance_[contact]) {
      // Touchdown: the world height the filter believes this contact landed at.
      const double measured = inekf_base_height + contact_relative_heights_[contact];
      // Only re-anchor on a genuine terrain change. Re-anchoring every touchdown would
      // fold the filter's own height noise into the reference and destroy the noise
      // rejection this term exists to provide; the deadband keeps flat ground pinned to
      // its exact anchor while still capturing a real step.
      if (std::abs(measured - contact_ground_anchor_[contact]) >
          settings_.height_anchor_update_threshold) {
        contact_ground_anchor_[contact] = measured;
      }
    }
    contact_was_in_stance_[contact] = in_stance;

    if (!in_stance) {
      continue;  // swing feet carry no ground information
    }
    const double ground =
      anchored ? contact_ground_anchor_[contact] : settings_.height_ground_z;
    sum += ground - contact_relative_heights_[contact];
    ++count;
  }

  if (count == 0) {
    return false;
  }
  height_out = sum / static_cast<double>(count);
  return true;
}

void InekfFloatingBaseEstimator::collectDiagnostics(const std::vector<bool>* foot_contacts)
{
  const size_t n = contact_frame_ids_.size();
  Diagnostics& d = diagnostics_;

  std::fill(d.contact_innovation.begin(), d.contact_innovation.end(), Eigen::Vector3d::Zero());
  std::fill(d.contact_innovation_std.begin(), d.contact_innovation_std.end(), Eigen::Vector3d::Zero());
  std::fill(d.contact_corrected.begin(), d.contact_corrected.end(), false);
  std::fill(d.contact_added.begin(), d.contact_added.end(), false);
  std::fill(d.contact_removed.begin(), d.contact_removed.end(), false);

  const auto& correction = estimator_->getLastCorrection();
  d.correction_applied = correction.applied;
  d.correction_dim = correction.dim;
  d.nis = correction.nis;
  d.nis_per_dof = correction.dim > 0 ? correction.nis / static_cast<double>(correction.dim) : 0.0;

  // The innovation is stacked 3 elements per corrected contact, in the order the
  // contacts appear in contact_ids; scatter it back onto the contact indexing.
  for (size_t k = 0; k < correction.contact_ids.size(); ++k) {
    const int id = correction.contact_ids[k];
    if (id < 0 || static_cast<size_t>(id) >= n) {
      continue;
    }
    const auto segment = static_cast<Eigen::Index>(3 * k);
    if (segment + 3 <= correction.innovation.size()) {
      d.contact_innovation[static_cast<size_t>(id)] = correction.innovation.segment<3>(segment);
      d.contact_innovation_std[static_cast<size_t>(id)] =
        correction.innovation_std.segment<3>(segment);
    }
    d.contact_corrected[static_cast<size_t>(id)] = true;
  }
  for (const int id : correction.added_contact_ids) {
    if (id >= 0 && static_cast<size_t>(id) < n) {
      d.contact_added[static_cast<size_t>(id)] = true;
    }
  }
  for (const int id : correction.removed_contact_ids) {
    if (id >= 0 && static_cast<size_t>(id) < n) {
      d.contact_removed[static_cast<size_t>(id)] = true;
    }
  }

  // Landmark positions the filter currently holds, for the contacts still augmented.
  const auto& inekf = estimator_->getInEKF();
  const auto& augmented = inekf.getEstimatedContactPositions();
  std::fill(d.contact_landmark_valid.begin(), d.contact_landmark_valid.end(), false);
  for (const auto& [id, index] : augmented) {
    if (id < 0 || static_cast<size_t>(id) >= n) {
      continue;
    }
    d.contact_landmark_position[static_cast<size_t>(id)] = inekf.getState().getVector(index);
    d.contact_landmark_valid[static_cast<size_t>(id)] = true;
  }
  d.num_augmented_contacts = estimator_->getNumAugmentedContacts();

  for (size_t contact = 0; contact < n; ++contact) {
    d.contact_in_stance[contact] = isContactInStance(contact, foot_contacts);
  }

  d.attitude_std = estimator_->getAttitudeStdDev();
  d.velocity_std = estimator_->getVelocityStdDev();
  d.position_std = estimator_->getPositionStdDev();
  d.gyroscope_bias_std = estimator_->getGyroscopeBiasStdDev();
  d.accelerometer_bias_std = estimator_->getAccelerometerBiasStdDev();

  d.gyroscope_bias = estimator_->getIMUGyroBiasEstimate();
  d.accelerometer_bias = estimator_->getIMULinearAccelerationBiasEstimate();
  d.estimating_bias = inekf.getEstimateBias();

  if (d.contact_ground_anchor.size() == contact_ground_anchor_.size()) {
    d.contact_ground_anchor = contact_ground_anchor_;
  }

  d.valid = true;
}

FloatingBaseEstimate InekfFloatingBaseEstimator::updateImpl(
  const Eigen::Vector3d& imu_gyro_body,
  const Eigen::Vector3d& imu_linear_acceleration_body,
  const std::vector<double>& joint_positions,
  const std::vector<double>& joint_velocities,
  const std::vector<double>& joint_efforts,
  const std::vector<bool>* foot_contacts)
{
  FloatingBaseEstimate estimate;
  if (!initialized_) {
    return estimate;
  }

  remapToEstimatorOrder(joint_positions, qj_);
  remapToEstimatorOrder(joint_velocities, dqj_);
  remapToEstimatorOrder(joint_efforts, tauj_);

  if (foot_contacts != nullptr) {
    std::vector<std::pair<int, bool>> contact_state;
    contact_state.reserve(settings_.contact_frames.size());
    for (size_t contact = 0; contact < settings_.contact_frames.size(); ++contact) {
      const int64_t foot = settings_.contact_foot_indices[contact];
      if (foot < 0 || static_cast<size_t>(foot) >= foot_contacts->size()) {
        throw std::invalid_argument(
          "[InekfFloatingBaseEstimator] contact_foot_indices contains an invalid foot index");
      }
      contact_state.emplace_back(
        static_cast<int>(contact), (*foot_contacts)[static_cast<size_t>(foot)]);
    }
    estimator_->update(
      imu_gyro_body, imu_linear_acceleration_body, qj_, dqj_, tauj_, contact_state);
  } else {
    estimator_->update(imu_gyro_body, imu_linear_acceleration_body, qj_, dqj_, tauj_);
  }

  // The InEKF estimates the IMU frame; convert back to the pelvis (floating-base) frame.
  const Eigen::Vector3d imu_position = estimator_->getBasePositionEstimate();
  const Eigen::Matrix3d imu_rotation = estimator_->getBaseRotationEstimate();
  const Eigen::Vector3d imu_linear_velocity_world = estimator_->getBaseLinearVelocityEstimateWorld();
  const Eigen::Vector3d imu_angular_velocity_local = estimator_->getBaseAngularVelocityEstimateLocal();

  const Eigen::Matrix3d base_rotation = imu_rotation * base_to_imu_rotation_.transpose();
  const Eigen::Vector3d base_position = imu_position - base_rotation * base_to_imu_translation_;
  // Rigid-body lever-arm correction for the linear velocity: v_base = v_imu + w x (p_base - p_imu).
  const Eigen::Vector3d angular_velocity_world = imu_rotation * imu_angular_velocity_local;
  const Eigen::Vector3d lever = base_position - imu_position;
  const Eigen::Vector3d base_linear_velocity_world =
    imu_linear_velocity_world + angular_velocity_world.cross(lever);

  estimate.position = base_position;
  estimate.orientation = Eigen::Quaterniond(base_rotation);

  // Base-height conditioning (see Settings::height_source). The MPC references foot
  // targets to an absolute ground plane, so the observation's height must agree with
  // the stance-foot kinematics rather than drift with the filter.
  // Keep the filter's own height before it is blended away: the blend happens
  // outside the filter, so the internal height keeps drifting uncorrected and the
  // touchdown anchors are derived from it. Only this value exposes that drift.
  diagnostics_.inekf_height = estimate.position.z();
  diagnostics_.kinematic_height_valid = false;
  diagnostics_.kinematic_height = 0.0;

  if (settings_.height_source != "inekf") {
    double kinematic_height = 0.0;
    if (kinematicBaseHeight(
        estimate.orientation, foot_contacts, estimate.position.z(), kinematic_height)) {
      diagnostics_.kinematic_height = kinematic_height;
      diagnostics_.kinematic_height_valid = true;
      if (settings_.height_source == "kinematic") {
        estimate.position.z() = kinematic_height;
      } else {  // "blend" and "anchored" both fuse with the filter height
        const double w = std::clamp(settings_.height_kinematic_weight, 0.0, 1.0);
        estimate.position.z() = (1.0 - w) * estimate.position.z() + w * kinematic_height;
      }
    }
  }
  diagnostics_.reported_height = estimate.position.z();

  estimate.linear_velocity_world = base_linear_velocity_world;
  estimate.angular_velocity_local = base_to_imu_rotation_ * imu_angular_velocity_local;
  estimate.valid = true;

  collectDiagnostics(foot_contacts);
  return estimate;
}

/******************************************************************************/

EstimatorType estimatorTypeFromString(const std::string & name)
{
  if (name == "inEKF") {
    return EstimatorType::InEkf;
  }
  if (name == "linearKF") {
    return EstimatorType::LinearKf;
  }
  throw std::invalid_argument(
    "[FloatingBaseEstimator] unknown stateEstimator.estimatorType '" + name +
    "'. Expected 'inEKF' or 'linearKF'.");
}

const char * toString(EstimatorType type)
{
  switch (type) {
    case EstimatorType::InEkf:
      return "inEKF";
    case EstimatorType::LinearKf:
      return "linearKF";
  }
  return "unknown";
}

std::unique_ptr<FloatingBaseEstimatorInterface> makeFloatingBaseEstimator(
  EstimatorType type, const EstimatorSettings & settings)
{
  switch (type) {
    case EstimatorType::InEkf:
      return std::make_unique<InekfFloatingBaseEstimator>(settings);

    case EstimatorType::LinearKf:
      return std::make_unique<LinearKfFloatingBaseEstimator>(settings);
  }
  throw std::invalid_argument("[FloatingBaseEstimator] unhandled EstimatorType");
}

}  // namespace legged_robot_mpc_controller::state_estimation
