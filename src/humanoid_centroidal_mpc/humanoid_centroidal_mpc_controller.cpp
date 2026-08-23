#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <Eigen/Geometry>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pluginlib/class_list_macros.hpp>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"
#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/centroidal_mpc_config_builder.hpp"

namespace legged_robot_mpc_controller
{

namespace
{

ocs2::vector_t make_vector(
  const std::vector<double>& values,
  const Eigen::Index expected_size,
  const double default_value,
  const std::string& name)
{
  if (values.empty()) {
    return ocs2::vector_t::Constant(expected_size, default_value);
  }
  if (static_cast<Eigen::Index>(values.size()) != expected_size) {
    throw std::invalid_argument(
      name + " size (" + std::to_string(values.size()) + ") must be empty or equal to " +
      std::to_string(expected_size));
  }
  return Eigen::Map<const ocs2::vector_t>(values.data(), expected_size);
}

std::optional<size_t> interface_offset(
  const std::vector<std::string>& interface_names,
  const std::string& interface_name)
{
  const auto it = std::find(interface_names.begin(), interface_names.end(), interface_name);
  if (it == interface_names.end()) {
    return std::nullopt;
  }
  return static_cast<size_t>(std::distance(interface_names.begin(), it));
}

bool uses_state_estimator(const std::string& source)
{
  return source.rfind("state_estimator", 0) == 0;
}

}  // namespace

HumanoidCentroidalMpcController::~HumanoidCentroidalMpcController()
{
  stop_solver_thread();
}

controller_interface::CallbackReturn HumanoidCentroidalMpcController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    parameters_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] init failed: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidCentroidalMpcController::on_configure(
  const rclcpp_lifecycle::State&)
{
  if (param_listener_->is_old(parameters_)) {
    parameters_ = param_listener_->get_params();
  }

  if (parameters_.paths.urdfFile.empty() || parameters_.paths.libFolder.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] paths.urdfFile or paths.libFolder is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build the centroidal MPC problem from ROS 2 parameters. The first run generates and
  // compiles the CppAD model libraries into paths.libFolder, which can take several
  // minutes; subsequent runs load the cached libraries.
  //
  // This block covers several independent subsystems, so the failure is reported
  // with the stage it happened in. Attributing every exception here to the MPC
  // interface (as this used to) sends you looking in the wrong subsystem: an
  // estimator or reference-manager failure would be reported as an MPC failure.
  const char* configure_stage = "building the centroidal MPC interface";
  try {
    mpc_interface_ =
      std::make_unique<ocs2::humanoid::CentroidalMpcInterface>(buildCentroidalMpcConfig(parameters_));
    control_model_.reset(dynamic_cast<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>*>(
      mpc_interface_->getMpcRobotModel().clone()));
    if (!control_model_) {
      throw std::runtime_error("failed to clone the centroidal MPC robot model");
    }
    control_pinocchio_ =
      std::make_unique<ocs2::PinocchioInterface>(mpc_interface_->getPinocchioInterface());

    const auto joint_dim = static_cast<Eigen::Index>(control_model_->getJointDim());
    mpc_joint_kp_ = make_vector(parameters_.control.mpcJointKp, joint_dim, 1200.0, "control.mpcJointKp");
    mpc_joint_kd_ = make_vector(parameters_.control.mpcJointKd, joint_dim, 10.0, "control.mpcJointKd");
    const auto fixed_joint_dim = static_cast<Eigen::Index>(parameters_.ocs2.model.fixedJointNames.size());
    fixed_joint_kp_ = make_vector(parameters_.control.fixedJointKp, fixed_joint_dim, 100.0, "control.fixedJointKp");
    fixed_joint_kd_ = make_vector(parameters_.control.fixedJointKd, fixed_joint_dim, 1.0, "control.fixedJointKd");

    configure_stage = "building the target-trajectories calculator / reference manager";
    const auto reference_config = common::buildReferenceConfig(parameters_);
    if (parameters_.ocs2.gait.gaitLibraryFile.empty()) {
      throw std::invalid_argument("[HumanoidCentroidalMpcController] ocs2.gait.gaitLibraryFile is empty.");
    }
    target_trajectories_calculator_ =
      std::make_unique<ocs2::humanoid::CentroidalMpcTargetTrajectoriesCalculator>(
      reference_config,
      mpc_interface_->getMpcRobotModel(),
      mpc_interface_->getPinocchioInterface(),
      mpc_interface_->getCentroidalModelInfo(),
      mpc_interface_->mpcSettings().timeHorizon_);
    auto velocity_target_to_target_trajectories =
      [this](
      const ocs2::vector4_t& velocity_target,
      ocs2::scalar_t init_time,
      ocs2::scalar_t /* final_time */,
      const ocs2::vector_t& init_state) {
        return target_trajectories_calculator_->commandedVelocityToTargetTrajectories(
          velocity_target,
          init_time,
          init_state);
      };
    auto base_pose_target_to_target_trajectories =
      [this](
      const ocs2::vector6_t& base_pose_target,
      ocs2::scalar_t init_time,
      ocs2::scalar_t /* final_time */,
      const ocs2::vector_t& init_state) {
        return target_trajectories_calculator_->commandedBasePoseToTargetTrajectories(
          base_pose_target, init_time, init_state);
      };
    auto motion_manager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      common::loadGaitMap(parameters_.ocs2.gait.gaitLibraryFile),
      reference_config,
      mpc_interface_->getSwitchedModelReferenceManagerPtr(),
      mpc_interface_->getMpcRobotModel(),
      std::move(velocity_target_to_target_trajectories),
      std::move(base_pose_target_to_target_trajectories),
      parameters_.target.defaultMode);
    if (!parameters_.ocs2.gait.stairClimbingFile.empty()) {
      motion_manager->setStairClimbingConfig(
        common::loadStairClimbingConfig(parameters_.ocs2.gait.stairClimbingFile));
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] stair climbing config loaded from %s",
        parameters_.ocs2.gait.stairClimbingFile.c_str());
    }
    if (!parameters_.ocs2.gait.terrainWalkingFile.empty()) {
      // Online terrain-aware walking (terrain_walk target mode): the terrain
      // model is built from the ground-truth geometry in this file, and the
      // planner settings come from its terrain_walk section.
      const auto terrain_geometry = common::loadTerrainStairsGeometry(parameters_.ocs2.gait.terrainWalkingFile);
      mpc_interface_->getSwitchedModelReferenceManagerPtr()->setTerrainFootholdPlanner(
        std::make_shared<ocs2::humanoid::TerrainFootholdPlanner>(
          ocs2::humanoid::buildTerrainModelFromStairs(terrain_geometry),
          common::loadTerrainFootholdPlannerSettings(parameters_.ocs2.gait.terrainWalkingFile)));
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] terrain walking config loaded from %s",
        parameters_.ocs2.gait.terrainWalkingFile.c_str());
    }
    motion_manager_ = std::move(motion_manager);
    rclcpp::QoS command_qos(1);
    command_qos.best_effort();
    motion_manager_->subscribe(
      get_node(), command_qos, parameters_.target.walkingVelocityTopic,
      parameters_.target.basePoseTopic, parameters_.target.modeTopic,
      parameters_.target.globalFrame);
    motion_manager_->subscribeMpcTargets(
      get_node(), parameters_.target.mpcTargetsTopic,
      common::selectAtIndices(
        parameters_.robot.jointNames,
        common::findArmJointIndices(parameters_.robot.jointNames)),
      parameters_.ocs2.costs.frameRelationTracking.sourceFrames,
      parameters_.ocs2.costs.frameRelationTracking.targetFrames);

    performance_visualization_ = std::make_unique<visualization::PerformanceVisualization>(
      get_node(),
      mpc_interface_->getPinocchioInterface(),
      mpc_interface_->getMpcRobotModel(),
      visualization::makePerformanceVisualizationSettings(parameters_));

    // Optional InEKF floating-base state estimator.
    configure_stage = "building the InEKF floating-base estimator";
    if (parameters_.stateEstimator.enabled ||
        uses_state_estimator(parameters_.floatingBase.source)) {
      const auto& se = parameters_.stateEstimator;
      state_estimation::InekfFloatingBaseEstimator::Settings settings;
      settings.urdf_path = parameters_.paths.urdfFile;
      settings.imu_frame = se.imuFrame;
      settings.contact_frames = se.contactFrames;
      settings.controller_joint_names = parameters_.robot.jointNames;
      settings.sampling_time = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));
      settings.gyroscope_noise = se.noise.gyroscope;
      settings.accelerometer_noise = se.noise.accelerometer;
      settings.gyroscope_bias_noise = se.noise.gyroscopeBias;
      settings.accelerometer_bias_noise = se.noise.accelerometerBias;
      settings.contact_noise = se.noise.contact;
      settings.initial_attitude_noise = se.initialCovariance.attitude;
      settings.initial_velocity_noise = se.initialCovariance.velocity;
      settings.initial_position_noise = se.initialCovariance.position;
      settings.initial_gyroscope_bias_noise = se.initialCovariance.gyroscopeBias;
      settings.initial_accelerometer_bias_noise = se.initialCovariance.accelerometerBias;
      settings.estimate_imu_bias = se.estimateImuBias;
      settings.contact_position_noise = se.noise.contactPosition;
      settings.contact_rotation_noise = se.noise.contactRotation;
      settings.contact_beta0 = se.contact.beta0;
      settings.contact_beta1 = se.contact.beta1;
      settings.contact_force_covariance_alpha = se.contact.forceCovarianceAlpha;
      settings.contact_probability_threshold = se.contact.probabilityThreshold;
      settings.dynamic_contact_estimation = se.contact.dynamicEstimation;
      settings.contact_source = se.contact.source;
      settings.contact_foot_indices = se.contact.footIndices;
      settings.height_source = se.height.source;
      settings.height_kinematic_weight = se.height.kinematicWeight;
      settings.height_ground_z = se.height.groundZ;
      settings.height_anchor_update_threshold = se.height.anchorUpdateThreshold;
      settings.lpf_gyro_cutoff = se.lpf.gyroCutoff;
      settings.lpf_gyro_accel_cutoff = se.lpf.gyroAccelCutoff;
      settings.lpf_lin_accel_cutoff = se.lpf.linAccelCutoff;
      settings.lpf_dqJ_cutoff = se.lpf.dqJCutoff;
      settings.lpf_ddqJ_cutoff = se.lpf.ddqJCutoff;
      settings.lpf_tauJ_cutoff = se.lpf.tauJCutoff;

      state_estimator_ =
        std::make_unique<state_estimation::InekfFloatingBaseEstimator>(settings);
      state_estimate_odom_publisher_ =
        get_node()->create_publisher<nav_msgs::msg::Odometry>(se.odomTopic, rclcpp::SystemDefaultsQoS());
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] InEKF state estimator enabled | imu=%s contacts=%zu joints=%d | "
        "update_rate=%u Hz sampling_time=%.6f s | contact_source=%s | drives_control=%s",
        se.imuFrame.c_str(), se.contactFrames.size(), state_estimator_->numEstimatorJoints(),
        get_update_rate(), settings.sampling_time, se.contact.source.c_str(),
        uses_state_estimator(parameters_.floatingBase.source) ? "yes" : "no");
    }
    configure_stage = "building the diagnostics logger";
    const auto& log = parameters_.diagnostics.log;
    if (log.enabled) {
      CentroidalMpcDiagnostics::Settings diagnostics_settings;
      diagnostics_settings.enabled = true;
      diagnostics_settings.pathPrefix = log.pathPrefix;
      diagnostics_settings.stateRate = log.stateRate;
      diagnostics_settings.logCost = log.logCost;
      diagnostics_settings.costRate = log.costRate;
      diagnostics_settings.bufferRows = static_cast<std::size_t>(std::max<int64_t>(log.bufferRows, 16));
      diagnostics_ = std::make_unique<CentroidalMpcDiagnostics>(
        diagnostics_settings, *mpc_interface_, *control_pinocchio_,
        parameters_.stateEstimator.contactFrames, parameters_.robot.jointNames);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] diagnostics logging enabled | prefix=%s | "
        "state=%.1f Hz cost=%s@%.1f Hz",
        log.pathPrefix.c_str(), log.stateRate, log.logCost ? "on" : "off", log.costRate);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] on_configure failed while %s: %s",
      configure_stage, e.what());
    // std::bad_array_new_length here almost always means a stale object file was
    // linked against a changed class layout rather than a bad parameter: an
    // allocation size is read from a member that is no longer where the caller
    // thinks it is. On a shared-folder workspace `make` can be fooled by clock
    // skew ("Clock skew detected. Your build may be incomplete."), so rebuild
    // both packages from scratch before treating it as a configuration problem.
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] if this is std::bad_array_new_length, remove "
      "build/ and install/ for legged_state_estimator and legged_robot_mpc_controller "
      "and rebuild: a partially rebuilt workspace mixes old code with a changed class "
      "layout.");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidCentroidalMpcController] configured | joints=%zu state_dim=%zu input_dim=%zu",
    parameters_.robot.jointNames.size(),
    static_cast<size_t>(control_model_->getStateDim()),
    static_cast<size_t>(control_model_->getInputDim()));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidCentroidalMpcController::on_activate(
  const rclcpp_lifecycle::State&)
{
  if (!mpc_interface_ || !control_model_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] activation requested before configuration.");
    return controller_interface::CallbackReturn::ERROR;
  }

  motion_manager_->requestVelocityTargetFilterReset();
  yaw_unwrapper_.reset();
  filtered_generalized_velocity_.resize(0);
  if (state_estimator_) {
    state_estimator_->reset();
    last_estimate_ = state_estimation::FloatingBaseEstimate{};
    last_estimate_publish_time_ = -1.0;
    estimator_warmup_end_time_ = -1.0;  // set on the first estimator tick
    estimator_driving_control_ = false;
    last_estimator_validation_time_ = -1.0;
    previous_gyroscope_bias_.setZero();
    previous_accelerometer_bias_.setZero();
    estimator_bias_validation_initialized_ = false;
  }
  if (diagnostics_) {
    try {
      diagnostics_->start();
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] diagnostics logs: %s | %s",
        diagnostics_->statePath().c_str(),
        diagnostics_->costPath().empty() ? "(cost log disabled)" : diagnostics_->costPath().c_str());
    } catch (const std::exception& e) {
      // Losing the log must not prevent the robot from running.
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] diagnostics logging disabled: %s", e.what());
      diagnostics_.reset();
    }
  }

  // Seed the mode-schedule snapshot before the solver thread exists, so the
  // first build_observation() has a schedule to read rather than defaulting to
  // STANCE.
  publish_mode_schedule();

  // Start from where the robot ACTUALLY is, not from ocs2.initialState. In
  // simulation the hardware pose comes from initial_pose.yaml through the
  // ros2_control xacro; on hardware it is simply wherever the robot is standing.
  // Seeding the MPC from a configured constant instead would make the first
  // solve plan from a pose the robot is not in, and the error would be silently
  // absorbed on the next tick rather than reported.
  //
  // State interfaces are only valid from on_activate onwards and the first
  // simulator publish can lag it slightly, so poll briefly rather than assuming
  // the very first read succeeds.
  ocs2::SystemObservation initial_observation;
  {
    constexpr auto kInitialStateTimeout = std::chrono::seconds(5);
    const auto deadline = std::chrono::steady_clock::now() + kInitialStateTimeout;
    while (rclcpp::ok()) {
      initial_observation = build_observation(get_node()->now());
      if (observation_from_hardware_) {
        break;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "[HumanoidCentroidalMpcController] joint and floating-base state interfaces did not "
          "become readable within 5 s; refusing to activate rather than starting the MPC from a "
          "configured pose the robot may not be in.");
        return controller_interface::CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  {
    const vector_t base_pose = control_model_->getBasePose(initial_observation.state);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] initial state read from hardware | base xyz=(%.3f, %.3f, "
      "%.3f) rpy_zyx=(%.3f, %.3f, %.3f)",
      base_pose(0), base_pose(1), base_pose(2), base_pose(3), base_pose(4), base_pose(5));
  }
  const vector_t q_hold = control_model_->getJointAngles(initial_observation.state);
  write_joint_action_command(
    q_hold,
    vector_t::Zero(q_hold.size()),
    compute_weight_compensating_torque(initial_observation));

  try {
    start_solver_thread(initial_observation);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] failed to start MPC solver: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !mrt_interface_->initialPolicyReceived()) {
    if ((std::chrono::steady_clock::now() - wait_start) > std::chrono::seconds(20)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] timed out waiting for the initial MPC policy.");
      stop_solver_thread();
      return controller_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidCentroidalMpcController] activated with initial MPC policy | joints=%zu",
    parameters_.robot.jointNames.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidCentroidalMpcController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  stop_solver_thread();
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  if (diagnostics_) {
    // The solver thread is already joined, so no more cost rows can arrive.
    const std::size_t dropped_state = diagnostics_->droppedStateRows();
    const std::size_t dropped_cost = diagnostics_->droppedCostRows();
    diagnostics_->stop();
    if (dropped_state > 0 || dropped_cost > 0) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] diagnostics log has gaps: dropped %zu state and %zu "
        "cost rows (writer fell behind); raise diagnostics.log.bufferRows or lower the rates.",
        dropped_state, dropped_cost);
    }
    if (diagnostics_->schemaMismatch()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController] diagnostics rows do not match the declared column "
        "count; the CSV columns are misaligned and must not be trusted.");
    }
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] diagnostics logs closed: %s",
      diagnostics_->statePath().c_str());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HumanoidCentroidalMpcController::command_interface_configuration() const
{
  const bool effort_pd = parameters_.robot.commandInterface == "effort_pd";
  const std::vector<std::string> command_ifaces = effort_pd ?
    std::vector<std::string>{"position", "velocity", "effort"} :
    std::vector<std::string>{parameters_.robot.commandInterface};
  auto config = make_joint_interface_configuration(command_ifaces);
  for (const auto& joint_name : parameters_.ocs2.model.fixedJointNames) {
    for (const auto& iface : command_ifaces) {
      config.names.emplace_back(joint_name + "/" + iface);
    }
  }
  if (!config.names.empty()) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  }
  return config;
}

controller_interface::InterfaceConfiguration
HumanoidCentroidalMpcController::state_interface_configuration() const
{
  auto config = make_joint_interface_configuration(parameters_.robot.stateInterfaces);

  for (const auto& joint_name : parameters_.ocs2.model.fixedJointNames) {
    for (const auto& interface_name : parameters_.robot.stateInterfaces) {
      if (!interface_name.empty()) {
        config.names.emplace_back(joint_name + "/" + interface_name);
      }
    }
  }

  const bool estimator_active = parameters_.stateEstimator.enabled ||
    uses_state_estimator(parameters_.floatingBase.source);
  // The GT body interfaces are still claimed with the estimator active, to bootstrap
  // the filter at activation and to publish a GT reference for comparison.
  const bool need_gt_interfaces =
    parameters_.floatingBase.source == "ground_truth_state" || estimator_active;

  if (need_gt_interfaces || estimator_active) {
    if (config.type == controller_interface::interface_configuration_type::NONE) {
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    }
  }

  if (need_gt_interfaces) {
    const auto floating_base_interfaces = floating_base_state_interface_names();
    config.names.insert(
      config.names.end(),
      floating_base_interfaces.begin(),
      floating_base_interfaces.end());
  }

  if (estimator_active) {
    // Joint torques for the torque-based contact estimator (unless already claimed
    // through robot.stateInterfaces).
    const bool effort_already_claimed = std::find(
      parameters_.robot.stateInterfaces.begin(),
      parameters_.robot.stateInterfaces.end(),
      hardware_interface::HW_IF_EFFORT) != parameters_.robot.stateInterfaces.end();
    if (!effort_already_claimed) {
      for (const auto& joint_name : parameters_.robot.jointNames) {
        config.names.emplace_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      }
    }
    // Pelvis IMU raw gyro + accelerometer.
    const std::string& imu = parameters_.stateEstimator.imuInterfaceName;
    for (const char* axis : {"x", "y", "z"}) {
      config.names.emplace_back(imu + "/angular_velocity." + axis);
      config.names.emplace_back(imu + "/linear_acceleration." + axis);
    }
  }

  return config;
}

controller_interface::return_type HumanoidCentroidalMpcController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type HumanoidCentroidalMpcController::update_and_write_commands(
  const rclcpp::Time& time,
  const rclcpp::Duration&)
{
  if (!mpc_interface_ || !control_model_) {
    return controller_interface::return_type::ERROR;
  }

  update_state_estimator(time);

  const auto observation = build_observation(time);
  JointActionCommand command;
  if (mrt_interface_ && mrt_interface_->initialPolicyReceived()) {
    command = compute_mpc_joint_action(observation);
  } else {
    // Hold the measured posture while waiting for the first policy.
    command.policy_position = control_model_->getJointAngles(observation.state);
    command.policy_velocity = vector_t::Zero(command.policy_position.size());
    command.feedforward = compute_weight_compensating_torque(observation);
  }

  write_joint_action_command(command.policy_position, command.policy_velocity, command.feedforward);

  // After the command exists, so the log carries the measured and the commanded
  // joint state from the same tick.
  record_diagnostics_state(time, observation, command);

  // 10 Hz hand-off: copying the policy trajectory every RT update starves the solver.
  if (performance_visualization_ && mrt_interface_ && mrt_interface_->initialPolicyReceived() &&
      (last_visualization_time_ < 0.0 || observation.time - last_visualization_time_ >= 0.1)) {
    last_visualization_time_ = observation.time;
    performance_visualization_->update_visualization(mrt_interface_->getPolicy().stateTrajectory_);
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
HumanoidCentroidalMpcController::on_export_reference_interfaces()
{
  // controller_manager rejects chainable controllers with zero reference interfaces,
  // so export a single dummy one (same pattern as dynamics_mpc_controller).
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.emplace_back(
    std::string(get_node()->get_name()),
    std::string("dummy_humanoid_centroidal_mpc/") + hardware_interface::HW_IF_EFFORT,
    reference_interfaces_.data());
  return reference_interfaces;
}

controller_interface::InterfaceConfiguration
HumanoidCentroidalMpcController::make_joint_interface_configuration(
  const std::vector<std::string>& interface_names) const
{
  controller_interface::InterfaceConfiguration config;
  if (parameters_.robot.jointNames.empty() || interface_names.empty()) {
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : parameters_.robot.jointNames) {
    for (const auto& interface_name : interface_names) {
      if (!interface_name.empty()) {
        config.names.emplace_back(joint_name + "/" + interface_name);
      }
    }
  }
  return config;
}

std::vector<std::string> HumanoidCentroidalMpcController::floating_base_state_interface_names() const
{
  const std::string& name = parameters_.floatingBase.stateInterfaceName;
  return {
    name + "/position.x",
    name + "/position.y",
    name + "/position.z",
    name + "/orientation.w",
    name + "/orientation.x",
    name + "/orientation.y",
    name + "/orientation.z",
    name + "/linear_velocity.x",
    name + "/linear_velocity.y",
    name + "/linear_velocity.z",
    name + "/angular_velocity.x",
    name + "/angular_velocity.y",
    name + "/angular_velocity.z",
  };
}

std::optional<double> HumanoidCentroidalMpcController::get_state_interface_value(
  const std::string& prefix_name,
  const std::string& interface_name) const
{
  const auto state_handle = std::find_if(
    state_interfaces_.begin(),
    state_interfaces_.end(),
    [&prefix_name, &interface_name](const auto& interface) {
      return interface.get_prefix_name() == prefix_name &&
             interface.get_interface_name() == interface_name;
    });
  if (state_handle == state_interfaces_.end()) {
    return std::nullopt;
  }
  return state_handle->get_value();
}

bool HumanoidCentroidalMpcController::read_joint_state(vector_t& q, vector_t& v)
{
  const size_t n = parameters_.robot.jointNames.size();
  const auto position_offset = interface_offset(
    parameters_.robot.stateInterfaces,
    hardware_interface::HW_IF_POSITION);
  const auto velocity_offset = interface_offset(
    parameters_.robot.stateInterfaces,
    hardware_interface::HW_IF_VELOCITY);
  if (!position_offset || !velocity_offset) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidCentroidalMpcController] stateInterfaces must include position and velocity.");
    return false;
  }

  const size_t stride = parameters_.robot.stateInterfaces.size();
  if (state_interfaces_.size() < n * stride) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidCentroidalMpcController] expected at least %zu state interfaces, got %zu.",
      n * stride,
      state_interfaces_.size());
    return false;
  }

  q.resize(static_cast<Eigen::Index>(n));
  v.resize(static_cast<Eigen::Index>(n));
  for (size_t i = 0; i < n; ++i) {
    q[static_cast<Eigen::Index>(i)] =
      state_interfaces_[i * stride + *position_offset].get_value();
    v[static_cast<Eigen::Index>(i)] =
      state_interfaces_[i * stride + *velocity_offset].get_value();
  }
  return true;
}

void HumanoidCentroidalMpcController::publish_mode_schedule()
{
  // Solver thread only, and only between iterations: the reference manager swaps
  // its buffered schedule in during preSolverRun(), so this is the point at
  // which reading it is not concurrent with that swap.
  if (const auto reference_manager = mpc_interface_->getSwitchedModelReferenceManagerPtr()) {
    mode_schedule_buffer_.writeFromNonRT(reference_manager->getModeSchedule());
  }
}

ocs2::contact_flag_t HumanoidCentroidalMpcController::contact_flags_at(double time)
{
  return ocs2::humanoid::modeNumber2StanceLeg(mode_at(time));
}

size_t HumanoidCentroidalMpcController::mode_at(double time)
{
  const auto* schedule = mode_schedule_buffer_.readFromRT();
  if (schedule == nullptr || schedule->modeSequence.empty()) {
    return ocs2::humanoid::STANCE;  // nothing published yet: assume double support
  }
  return schedule->modeAtTime(time);
}

void HumanoidCentroidalMpcController::update_state_estimator(const rclcpp::Time& time)
{
  if (!state_estimator_) {
    return;
  }

  // Joint states in the controller's jointNames order (torque optional -> 0).
  const auto& joint_names = parameters_.robot.jointNames;
  std::vector<double> joint_pos(joint_names.size());
  std::vector<double> joint_vel(joint_names.size());
  std::vector<double> joint_eff(joint_names.size(), 0.0);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto p = get_state_interface_value(joint_names[i], hardware_interface::HW_IF_POSITION);
    const auto v = get_state_interface_value(joint_names[i], hardware_interface::HW_IF_VELOCITY);
    if (!p || !v) {
      return;  // joints not ready yet
    }
    joint_pos[i] = *p;
    joint_vel[i] = *v;
    if (const auto e = get_state_interface_value(joint_names[i], hardware_interface::HW_IF_EFFORT)) {
      joint_eff[i] = *e;
    }
  }

  // Raw pelvis IMU (body-local gyro + accelerometer).
  const std::string& imu = parameters_.stateEstimator.imuInterfaceName;
  const auto gx = get_state_interface_value(imu, "angular_velocity.x");
  const auto gy = get_state_interface_value(imu, "angular_velocity.y");
  const auto gz = get_state_interface_value(imu, "angular_velocity.z");
  const auto ax = get_state_interface_value(imu, "linear_acceleration.x");
  const auto ay = get_state_interface_value(imu, "linear_acceleration.y");
  const auto az = get_state_interface_value(imu, "linear_acceleration.z");
  if (!gx || !gy || !gz || !ax || !ay || !az) {
    return;  // IMU not ready yet
  }
  const Eigen::Vector3d gyro(*gx, *gy, *gz);
  const Eigen::Vector3d accel(*ax, *ay, *az);

  // Bootstrap the filter from the MuJoCo ground-truth body pose on the first tick.
  if (!state_estimator_->initialized()) {
    const std::string& base = parameters_.floatingBase.stateInterfaceName;
    const auto px = get_state_interface_value(base, "position.x");
    const auto py = get_state_interface_value(base, "position.y");
    const auto pz = get_state_interface_value(base, "position.z");
    const auto qw = get_state_interface_value(base, "orientation.w");
    const auto qx = get_state_interface_value(base, "orientation.x");
    const auto qy = get_state_interface_value(base, "orientation.y");
    const auto qz = get_state_interface_value(base, "orientation.z");
    if (!px || !py || !pz || !qw || !qx || !qy || !qz) {
      return;
    }
    Eigen::Quaterniond quat(*qw, *qx, *qy, *qz);
    if (quat.norm() < 1e-9) {
      quat = Eigen::Quaterniond::Identity();
    }
    state_estimator_->initialize(
      Eigen::Vector3d(*px, *py, *pz), quat.normalized(), joint_pos);
    // Start the convergence window from the first estimator tick.
    estimator_warmup_end_time_ =
      time.seconds() + std::max(0.0, parameters_.stateEstimator.warmupSeconds);
  }

  if (parameters_.stateEstimator.contact.source == "scheduled") {
    // From the control-thread snapshot, not the reference manager directly: see
    // mode_schedule_buffer_. A torn read here would hand the filter a wrong
    // contact flag and add or drop a landmark at the wrong instant.
    const auto contact_flags = contact_flags_at(time.seconds());
    last_estimate_ = state_estimator_->update(
      gyro, accel, joint_pos, joint_vel, joint_eff,
      std::vector<bool>{contact_flags[0], contact_flags[1]});
  } else {
    last_estimate_ = state_estimator_->update(
      gyro, accel, joint_pos, joint_vel, joint_eff);
  }


  // Publish the estimate as odometry for evaluation against the GT body frame.
  if (state_estimate_odom_publisher_ && last_estimate_.valid) {
    const double t = time.seconds();
    const double period = 1.0 / std::max(1.0, parameters_.stateEstimator.publishRate);
    if (last_estimate_publish_time_ < 0.0 || t - last_estimate_publish_time_ >= period) {
      last_estimate_publish_time_ = t;
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = time;
      odom.header.frame_id = parameters_.stateEstimator.odomFrame;
      odom.child_frame_id = parameters_.floatingBase.baseFrame;
      odom.pose.pose.position.x = last_estimate_.position.x();
      odom.pose.pose.position.y = last_estimate_.position.y();
      odom.pose.pose.position.z = last_estimate_.position.z();
      odom.pose.pose.orientation.w = last_estimate_.orientation.w();
      odom.pose.pose.orientation.x = last_estimate_.orientation.x();
      odom.pose.pose.orientation.y = last_estimate_.orientation.y();
      odom.pose.pose.orientation.z = last_estimate_.orientation.z();
      // Twist in the body frame (odometry child-frame convention).
      const Eigen::Vector3d v_body =
        last_estimate_.orientation.conjugate() * last_estimate_.linear_velocity_world;
      odom.twist.twist.linear.x = v_body.x();
      odom.twist.twist.linear.y = v_body.y();
      odom.twist.twist.linear.z = v_body.z();
      odom.twist.twist.angular.x = last_estimate_.angular_velocity_local.x();
      odom.twist.twist.angular.y = last_estimate_.angular_velocity_local.y();
      odom.twist.twist.angular.z = last_estimate_.angular_velocity_local.z();
      state_estimate_odom_publisher_->publish(odom);
    }
  }

  log_state_estimator_validation(time);
}

void HumanoidCentroidalMpcController::log_state_estimator_validation(
  const rclcpp::Time& time)
{
  const auto& validation = parameters_.stateEstimator.validation;
  if (!validation.enabled || !state_estimator_ || !last_estimate_.valid) {
    return;
  }

  const double now = time.seconds();
  if (last_estimator_validation_time_ >= 0.0 &&
      now - last_estimator_validation_time_ < validation.period) {
    return;
  }
  const double validation_dt = last_estimator_validation_time_ >= 0.0 ?
    now - last_estimator_validation_time_ : 0.0;
  last_estimator_validation_time_ = now;

  const std::string& base = parameters_.floatingBase.stateInterfaceName;
  const auto qw = get_state_interface_value(base, "orientation.w");
  const auto qx = get_state_interface_value(base, "orientation.x");
  const auto qy = get_state_interface_value(base, "orientation.y");
  const auto qz = get_state_interface_value(base, "orientation.z");
  const auto lvx = get_state_interface_value(base, "linear_velocity.x");
  const auto lvy = get_state_interface_value(base, "linear_velocity.y");
  const auto lvz = get_state_interface_value(base, "linear_velocity.z");
  const auto avx = get_state_interface_value(base, "angular_velocity.x");
  const auto avy = get_state_interface_value(base, "angular_velocity.y");
  const auto avz = get_state_interface_value(base, "angular_velocity.z");
  if (!qw || !qx || !qy || !qz || !lvx || !lvy || !lvz || !avx || !avy || !avz) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController][INEKF_VELOCITY_VALIDATION] ground-truth "
      "floating-base interfaces are unavailable; validation skipped.");
    return;
  }

  Eigen::Quaterniond gt_orientation(*qw, *qx, *qy, *qz);
  if (!gt_orientation.coeffs().allFinite() || gt_orientation.norm() < 1.0e-12) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController][INEKF_VELOCITY_VALIDATION] invalid "
      "ground-truth orientation; validation skipped.");
    return;
  }
  gt_orientation.normalize();

  // MuJoCo exposes both twist components in the pelvis frame. Compare linear
  // velocity in world coordinates and angular velocity in the pelvis frame,
  // matching the representation consumed by build_observation().
  const Eigen::Vector3d gt_linear_velocity_world =
    gt_orientation * Eigen::Vector3d(*lvx, *lvy, *lvz);
  const Eigen::Vector3d gt_angular_velocity_local(*avx, *avy, *avz);
  const Eigen::Vector3d linear_velocity_error =
    last_estimate_.linear_velocity_world - gt_linear_velocity_world;
  const Eigen::Vector3d angular_velocity_error =
    last_estimate_.angular_velocity_local - gt_angular_velocity_local;

  const Eigen::Vector3d gyroscope_bias = state_estimator_->estimatedGyroscopeBias();
  const Eigen::Vector3d accelerometer_bias = state_estimator_->estimatedAccelerometerBias();
  double gyroscope_bias_rate = 0.0;
  double accelerometer_bias_rate = 0.0;
  if (estimator_bias_validation_initialized_ && validation_dt > 0.0) {
    gyroscope_bias_rate =
      (gyroscope_bias - previous_gyroscope_bias_).norm() / validation_dt;
    accelerometer_bias_rate =
      (accelerometer_bias - previous_accelerometer_bias_).norm() / validation_dt;
  }
  previous_gyroscope_bias_ = gyroscope_bias;
  previous_accelerometer_bias_ = accelerometer_bias;
  estimator_bias_validation_initialized_ = true;

  const double linear_error_norm = linear_velocity_error.norm();
  const double angular_error_norm = angular_velocity_error.norm();
  const bool finite = last_estimate_.linear_velocity_world.allFinite() &&
    last_estimate_.angular_velocity_local.allFinite() && linear_velocity_error.allFinite() &&
    angular_velocity_error.allFinite() && gyroscope_bias.allFinite() &&
    accelerometer_bias.allFinite();
  const bool valid = finite &&
    linear_error_norm <= validation.maxLinearVelocityError &&
    angular_error_norm <= validation.maxAngularVelocityError;
  const char* control_source = estimator_driving_control_ ? "state_estimator" : "ground_truth_state";

  std::ostringstream message;
  message << std::fixed << std::setprecision(4)
          << "control=" << control_source << " finite=" << (finite ? "true" : "false")
          << " | v_est_W=(" << last_estimate_.linear_velocity_world.transpose() << ")"
          << " v_gt_W=(" << gt_linear_velocity_world.transpose() << ")"
          << " e_v=(" << linear_velocity_error.transpose() << ")"
          << " |e_v|=" << linear_error_norm << "/" << validation.maxLinearVelocityError << " m/s"
          << " | omega_est_B=(" << last_estimate_.angular_velocity_local.transpose() << ")"
          << " omega_gt_B=(" << gt_angular_velocity_local.transpose() << ")"
          << " e_omega=(" << angular_velocity_error.transpose() << ")"
          << " |e_omega|=" << angular_error_norm << "/" << validation.maxAngularVelocityError << " rad/s"
          << std::setprecision(5)
          << " | b_g=(" << gyroscope_bias.transpose() << ")"
          << " b_a=(" << accelerometer_bias.transpose() << ")"
          << std::scientific << std::setprecision(3)
          << " |db_g/dt|=" << gyroscope_bias_rate
          << " |db_a/dt|=" << accelerometer_bias_rate;
  const std::string message_text = message.str();
  if (valid) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController][INEKF_VELOCITY_VALIDATION][PASS] %s",
      message_text.c_str());
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController][INEKF_VELOCITY_VALIDATION][FAIL] %s",
      message_text.c_str());
  }
}

void HumanoidCentroidalMpcController::record_diagnostics_state(
  const rclcpp::Time& time,
  const ocs2::SystemObservation& observation,
  const JointActionCommand& command)
{
  if (!diagnostics_) {
    return;
  }

  // Hand the observation to the solver thread for the cost breakdown. try_lock
  // so a slow reader can never stall the control loop; a skipped tick just means
  // the solver evaluates the previous observation.
  {
    std::unique_lock<std::mutex> lock(diagnostics_observation_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      diagnostics_observation_ = observation;
      diagnostics_observation_valid_ = true;
    }
  }

  if (!state_estimator_) {
    return;  // the estimator log has nothing to say without an estimator
  }

  auto& sample = diagnostics_sample_;
  sample.time = time.seconds();
  // Phase segments the run so warm-up, hand-off and steady state can be analysed
  // separately without having to guess where the transitions happened.
  sample.phase = !state_estimator_->initialized() ? 0 : (estimator_driving_control_ ? 2 : 1);
  sample.mode = static_cast<int>(observation.mode);

  const auto contact_flags = contact_flags_at(sample.time);
  sample.contact_left = contact_flags[0];
  sample.contact_right = contact_flags[1];

  const std::string& base = parameters_.floatingBase.stateInterfaceName;
  const auto px = get_state_interface_value(base, "position.x");
  const auto py = get_state_interface_value(base, "position.y");
  const auto pz = get_state_interface_value(base, "position.z");
  const auto qw = get_state_interface_value(base, "orientation.w");
  const auto qx = get_state_interface_value(base, "orientation.x");
  const auto qy = get_state_interface_value(base, "orientation.y");
  const auto qz = get_state_interface_value(base, "orientation.z");
  const auto lvx = get_state_interface_value(base, "linear_velocity.x");
  const auto lvy = get_state_interface_value(base, "linear_velocity.y");
  const auto lvz = get_state_interface_value(base, "linear_velocity.z");
  const auto avx = get_state_interface_value(base, "angular_velocity.x");
  const auto avy = get_state_interface_value(base, "angular_velocity.y");
  const auto avz = get_state_interface_value(base, "angular_velocity.z");
  sample.gt_valid = px && py && pz && qw && qx && qy && qz && lvx && lvy && lvz && avx && avy && avz;
  if (sample.gt_valid) {
    sample.gt_position = Eigen::Vector3d(*px, *py, *pz);
    Eigen::Quaterniond gt_orientation(*qw, *qx, *qy, *qz);
    if (gt_orientation.norm() < 1e-12) {
      gt_orientation = Eigen::Quaterniond::Identity();
    } else {
      gt_orientation.normalize();
    }
    sample.gt_orientation = gt_orientation;
    // MuJoCo reports the GT twist body-local; match build_observation and store
    // the linear part in world coordinates.
    sample.gt_linear_velocity_world = gt_orientation * Eigen::Vector3d(*lvx, *lvy, *lvz);
    sample.gt_angular_velocity_local = Eigen::Vector3d(*avx, *avy, *avz);
  }

  // The joint state is shared by both momentum evaluations, so the two differ
  // only in the floating-base feedback - which is the comparison we want.
  sample.joint_valid = read_joint_state(sample.joint_positions, sample.joint_velocities);

  sample.commanded_positions = command.policy_position;
  sample.commanded_velocities = command.policy_velocity;
  sample.commanded_torques = command.feedforward;
  sample.command_valid = command.policy_position.size() > 0;

  diagnostics_->recordState(sample, last_estimate_, state_estimator_->diagnostics());
}

ocs2::SystemObservation HumanoidCentroidalMpcController::build_observation(const rclcpp::Time& time)
{
  const auto& info = mpc_interface_->getCentroidalModelInfo();
  const auto joint_dim = static_cast<Eigen::Index>(control_model_->getJointDim());
  const auto gen_coordinates_dim = static_cast<Eigen::Index>(info.generalizedCoordinatesNum);

  observation_from_hardware_ = false;

  ocs2::SystemObservation observation;
  observation.time = time.seconds();
  // Every element of this is overwritten from the hardware interfaces below, so
  // the seed only matters if a read fails - and in that case holding the last
  // KNOWN state is right, while asserting the configured ocs2.initialState would
  // claim the robot is somewhere it may well not be. The configured value is a
  // construction-time placeholder for the interface, not a runtime pose.
  observation.state = has_hardware_observation_ ?
    last_hardware_observation_.state :
    mpc_interface_->getInitialState();
  observation.input = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getInputDim()));
  observation.mode = mode_at(observation.time);

  vector_t q_joint;
  vector_t v_joint;
  if (!read_joint_state(q_joint, v_joint)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidCentroidalMpcController] missing joint state interfaces; holding last observation.");
    return hold_last_hardware_observation(observation);
  }

  // Floating-base state: world-frame position/orientation/linear velocity and
  // body-local angular velocity, from either the InEKF estimate or the GT body.
  ocs2::vector3_t base_position;
  Eigen::Quaterniond base_orientation;
  ocs2::vector3_t linear_velocity_world;
  ocs2::vector3_t angular_velocity_local;

  // The InEKF is seeded with zero velocity and unknown IMU biases, so its first
  // seconds carry phantom base velocity. The centroidal state is normalized
  // momentum (A(q) v / m), so that error enters the dominant state directly and
  // destabilizes the robot. Keep using the state-interface base until the filter
  // has converged.
  const bool warmed_up = estimator_warmup_end_time_ >= 0.0 &&
    time.seconds() >= estimator_warmup_end_time_;
  const bool use_estimate =
    uses_state_estimator(parameters_.floatingBase.source) && last_estimate_.valid && warmed_up;
  if (use_estimate && !estimator_driving_control_) {
    estimator_driving_control_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] state estimator warm-up complete; the InEKF estimate now drives control.");
  }
  const std::string& base_name = parameters_.floatingBase.stateInterfaceName;
  const auto px = get_state_interface_value(base_name, "position.x");
  const auto py = get_state_interface_value(base_name, "position.y");
  const auto pz = get_state_interface_value(base_name, "position.z");
  const auto qw = get_state_interface_value(base_name, "orientation.w");
  const auto qx = get_state_interface_value(base_name, "orientation.x");
  const auto qy = get_state_interface_value(base_name, "orientation.y");
  const auto qz = get_state_interface_value(base_name, "orientation.z");
  const auto lvx = get_state_interface_value(base_name, "linear_velocity.x");
  const auto lvy = get_state_interface_value(base_name, "linear_velocity.y");
  const auto lvz = get_state_interface_value(base_name, "linear_velocity.z");
  const auto avx = get_state_interface_value(base_name, "angular_velocity.x");
  const auto avy = get_state_interface_value(base_name, "angular_velocity.y");
  const auto avz = get_state_interface_value(base_name, "angular_velocity.z");
  if (!px || !py || !pz || !qw || !qx || !qy || !qz || !lvx || !lvy || !lvz || !avx || !avy ||
      !avz) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidCentroidalMpcController] missing floating-base interfaces; holding last observation.");
    return hold_last_hardware_observation(observation);
  }
  base_position = ocs2::vector3_t(*px, *py, *pz);
  base_orientation = Eigen::Quaterniond(*qw, *qx, *qy, *qz);
  // The GT twist is body-local; rotate the linear part into the world frame.
  const Eigen::Quaterniond gt_orientation =
    base_orientation.norm() < 1e-12 ? Eigen::Quaterniond::Identity() : base_orientation.normalized();
  linear_velocity_world = gt_orientation * ocs2::vector3_t(*lvx, *lvy, *lvz);
  angular_velocity_local = ocs2::vector3_t(*avx, *avy, *avz);

  if (use_estimate) {
    // Full InEKF feedback: pose, world linear velocity and body angular velocity all
    // come from the estimator, so the control loop no longer reads the ground-truth body.
    base_position = last_estimate_.position;
    base_orientation = last_estimate_.orientation;
    linear_velocity_world = last_estimate_.linear_velocity_world;
    angular_velocity_local = last_estimate_.angular_velocity_local;
  }

  if (base_orientation.norm() < 1e-12) {
    base_orientation = Eigen::Quaterniond::Identity();
  } else {
    base_orientation.normalize();
  }
  ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(base_orientation);
  euler_zyx(0) = yaw_unwrapper_.unwrap(euler_zyx(0));

  // Generalized coordinates and velocities in the pinocchio (centroidal) convention.
  vector_t q_pinocchio(gen_coordinates_dim);
  q_pinocchio.head<3>() = base_position;
  q_pinocchio.segment<3>(3) = euler_zyx;
  q_pinocchio.tail(joint_dim) = q_joint;

  vector_t v_pinocchio(gen_coordinates_dim);
  v_pinocchio.head<3>() = linear_velocity_world;
  v_pinocchio.segment<3>(3) =
    ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(
    euler_zyx, angular_velocity_local);
  v_pinocchio.tail(joint_dim) = v_joint;

  // Low-pass the generalized velocities: raw simulator velocity dither at node 0 keeps
  // the SQP baseline defect above g_max and traps the linesearch in constraint mode.
  const double cutoff_hz = parameters_.control.observationVelocityFilterCutoffHz;
  if (cutoff_hz > 0.0) {
    const double dt = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt);
    if (filtered_generalized_velocity_.size() != v_pinocchio.size()) {
      filtered_generalized_velocity_ = v_pinocchio;
    } else {
      filtered_generalized_velocity_ =
        alpha * v_pinocchio + (1.0 - alpha) * filtered_generalized_velocity_;
    }
    v_pinocchio = filtered_generalized_velocity_;
  }

  // Centroidal state: [normalized momentum (6), generalized coordinates].
  //
  // Provenance of every element, which is the whole point of this function:
  //   momentum (6)   A_G(q) v / m, so it follows the floating-base twist below
  //   base pose (6)  from the floating-base source - the InEKF once the warm-up
  //                  window has passed, the simulator/hardware body before that
  //   joints (23)    straight from the ros2_control state interfaces
  // Nothing here comes from ocs2.initialState; that value is only a
  // construction-time placeholder for the interface.
  ocs2::updateCentroidalDynamics(*control_pinocchio_, info, q_pinocchio);
  const auto& momentum_matrix = ocs2::getCentroidalMomentumMatrix(*control_pinocchio_);
  ocs2::centroidal_model::getNormalizedMomentum(observation.state, info).noalias() =
    momentum_matrix * v_pinocchio / info.robotMass;
  ocs2::centroidal_model::getGeneralizedCoordinates(observation.state, info) = q_pinocchio;

  // Keep the observed input consistent with the velocity used to construct momentum.
  // Contact wrenches remain unknown (zero); measured joint velocities populate the input tail.
  const vector_t filtered_joint_velocity = v_pinocchio.tail(joint_dim);
  control_model_->setJointVelocities(
    observation.state, observation.input, filtered_joint_velocity);

  observation_from_hardware_ = true;
  last_hardware_observation_ = observation;
  has_hardware_observation_ = true;
  return observation;
}

ocs2::SystemObservation HumanoidCentroidalMpcController::hold_last_hardware_observation(
  const ocs2::SystemObservation& fallback) const
{
  if (!has_hardware_observation_) {
    return fallback;
  }
  // Keep the last state that was actually measured, but advance its timestamp so
  // the policy is still evaluated at the current time rather than replayed.
  ocs2::SystemObservation held = last_hardware_observation_;
  held.time = fallback.time;
  held.mode = fallback.mode;
  return held;
}

ocs2::TargetTrajectories HumanoidCentroidalMpcController::current_observation_to_reset_trajectory(
  const ocs2::SystemObservation& observation)
{
  vector_t target_state = observation.state;
  // Zero out the momentum and the pitch/roll angles.
  target_state.head<6>().setZero();
  target_state.segment<2>(10).setZero();
  return ocs2::TargetTrajectories(
    {observation.time},
    {target_state},
    {vector_t::Zero(observation.input.size())});
}

void HumanoidCentroidalMpcController::start_solver_thread(
  const ocs2::SystemObservation& initial_observation)
{
  stop_solver_thread();

  const std::string& solver_type = parameters_.ocs2.mpc.solverType;
  if (solver_type != "sqp") {
    throw std::runtime_error(
      "[HumanoidCentroidalMpcController] unsupported solver type: " + solver_type);
  }
  mpc_solver_ = std::make_unique<ocs2::SqpMpc>(
    mpc_interface_->mpcSettings(),
    mpc_interface_->sqpSettings(),
    mpc_interface_->getOptimalControlProblem(),
    mpc_interface_->getInitializer());

  mpc_solver_->getSolverPtr()->setReferenceManager(mpc_interface_->getReferenceManagerPtr());
  mrt_interface_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_solver_);
  mrt_interface_->initRollout(&mpc_interface_->getRollout());
  mrt_interface_->setCurrentObservation(initial_observation);
  mrt_interface_->resetMpcNode(current_observation_to_reset_trajectory(initial_observation));
  mpc_solver_->getSolverPtr()->addSynchronizedModule(motion_manager_);

  terminate_solver_thread_.store(false);
  solver_thread_ = std::jthread([this] { solver_worker(); });
}

void HumanoidCentroidalMpcController::stop_solver_thread()
{
  terminate_solver_thread_.store(true);
  if (solver_thread_.joinable()) {
    solver_thread_.join();
  }
  mrt_interface_.reset();
  mpc_solver_.reset();
}

void HumanoidCentroidalMpcController::solver_worker()
{
  using Clock = std::chrono::steady_clock;
  const double target_frequency = static_cast<double>(parameters_.ocs2.mpc.mpcDesiredFrequency);
  const double target_period_ms = 1000.0 / target_frequency;
  const auto period = std::chrono::duration<double>(1.0 / target_frequency);
  const auto log_period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(parameters_.diagnostics.statusLogPeriod));

  Clock::time_point next_log = Clock::now() + log_period;
  double accumulated_period_ms = 0.0;
  double accumulated_advance_ms = 0.0;
  std::size_t period_samples = 0;
  std::size_t advance_samples = 0;
  Clock::time_point previous_start{};

  while (!terminate_solver_thread_.load() && rclcpp::ok()) {
    const auto iteration_start = Clock::now();
    const auto wakeup_time = iteration_start + period;
    if (previous_start != Clock::time_point{}) {
      accumulated_period_ms +=
        std::chrono::duration<double, std::milli>(iteration_start - previous_start).count();
      ++period_samples;
    }
    previous_start = iteration_start;

    double advance_ms = 0.0;
    bool policy_updated = false;
    try {
      mrt_interface_->advanceMpc();
      const auto iteration_end = Clock::now();
      advance_ms = std::chrono::duration<double, std::milli>(iteration_end - iteration_start).count();
      accumulated_advance_ms += advance_ms;
      ++advance_samples;

      // The solve is finished, so the reference manager's buffered schedule is
      // no longer being swapped; republish it for the control loop.
      publish_mode_schedule();

      policy_updated = mrt_interface_->updatePolicy();
      if (!policy_updated) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(),
          2000,
          "[HumanoidCentroidalMpcController] MPC solver failed to update policy.");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "[HumanoidCentroidalMpcController] MPC solver exception: %s",
        e.what());
    }

    // Per-cost-term breakdown, evaluated here rather than in the control thread:
    // the terms read the reference manager (mode schedule, swing trajectories),
    // which this thread writes in preSolverRun(). Between iterations is the only
    // point where reading it does not race with that write.
    if (diagnostics_) {
      ocs2::SystemObservation cost_observation;
      bool have_observation = false;
      {
        std::lock_guard<std::mutex> lock(diagnostics_observation_mutex_);
        if (diagnostics_observation_valid_) {
          cost_observation = diagnostics_observation_;
          have_observation = true;
        }
      }
      CentroidalMpcDiagnostics::SolverHealth health;
      if (mpc_solver_) {
        // Read on the solver thread, between iterations, where the solver is
        // idle. These say whether the returned solution actually satisfies the
        // dynamics and the constraints - a converged-but-wrong problem and a
        // non-converging solver look identical from the cost alone.
        try {
          const auto& performance = mpc_solver_->getSolverPtr()->getPerformanceIndeces();
          health.merit = performance.merit;
          health.cost = performance.cost;
          health.dynamics_violation_sse = performance.dynamicsViolationSSE;
          health.equality_constraints_sse = performance.equalityConstraintsSSE;
          health.dual_feasibilities_sse = performance.dualFeasibilitiesSSE;
          health.equality_lagrangian = performance.equalityLagrangian;
          health.inequality_lagrangian = performance.inequalityLagrangian;
          health.iterations =
            static_cast<int>(mpc_solver_->getSolverPtr()->getIterationsLog().size());
          health.policy_updated = policy_updated;
          health.valid = true;
        } catch (const std::exception&) {
          health.valid = false;  // no solution yet
        }
      }
      if (have_observation) {
        diagnostics_->recordCost(cost_observation, advance_ms, health);
      }
    }

    if (Clock::now() >= next_log && advance_samples > 0) {
      const double average_period_ms = period_samples > 0 ?
        accumulated_period_ms / static_cast<double>(period_samples) : target_period_ms;
      const double average_advance_ms =
        accumulated_advance_ms / static_cast<double>(advance_samples);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[HumanoidCentroidalMpcController][MPC_TIMING] target=%.2f Hz, actual=%.2f Hz, "
        "latest advance=%.3f ms, average advance=%.3f ms, target period=%.3f ms, utilization=%.1f%%.",
        target_frequency,
        1000.0 / average_period_ms,
        advance_ms,
        average_advance_ms,
        target_period_ms,
        100.0 * average_advance_ms / target_period_ms);
      accumulated_period_ms = 0.0;
      accumulated_advance_ms = 0.0;
      period_samples = 0;
      advance_samples = 0;
      next_log = Clock::now() + log_period;
    }

    std::this_thread::sleep_until(wakeup_time);
  }
}

HumanoidCentroidalMpcController::vector_t
HumanoidCentroidalMpcController::compute_weight_compensating_torque(
  const ocs2::SystemObservation& observation)
{
  const vector_t input = ocs2::humanoid::weightCompensatingInput(
    *control_pinocchio_, {true, true}, *control_model_);
  const std::array<ocs2::vector6_t, 2> foot_wrenches{
    control_model_->getContactWrench(input, 0), control_model_->getContactWrench(input, 1)};

  vector_t state = observation.state;
  vector_t zero_input = vector_t::Zero(observation.input.size());
  const vector_t q = control_model_->getGeneralizedCoordinates(state);
  const vector_t qd = control_model_->getGeneralizedVelocities(state, zero_input);
  const vector_t qdd_joints = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getJointDim()));
  return ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
    q, qd, qdd_joints, foot_wrenches, *control_pinocchio_);
}

HumanoidCentroidalMpcController::JointActionCommand
HumanoidCentroidalMpcController::compute_mpc_joint_action(const ocs2::SystemObservation& observation)
{
  mrt_interface_->setCurrentObservation(observation);

  JointActionCommand command;
  vector_t policy_state;
  vector_t policy_input;
  size_t policy_mode = ocs2::humanoid::STANCE;
  mrt_interface_->evaluatePolicy(
    observation.time + parameters_.control.policyTimeOffset,
    observation.state,
    policy_state,
    policy_input,
    policy_mode);

  command.policy_position = control_model_->getJointAngles(policy_state);
  command.policy_velocity = control_model_->getJointVelocities(policy_state, policy_input);

  // Legacy CentroidalMpcMrtJointController: RNEA feedforward with the policy contact
  // wrenches (its acceleration-level PD gains are zero, so qdd_des = 0).
  const std::array<ocs2::vector6_t, 2> foot_wrenches{
    control_model_->getContactWrench(policy_input, 0),
    control_model_->getContactWrench(policy_input, 1)};

  vector_t state = observation.state;
  vector_t input = observation.input;
  const vector_t q = control_model_->getGeneralizedCoordinates(state);
  const vector_t qd = control_model_->getGeneralizedVelocities(state, input);
  const vector_t qdd_joints = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getJointDim()));
  command.feedforward = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
    q, qd, qdd_joints, foot_wrenches, *control_pinocchio_);
  return command;
}

void HumanoidCentroidalMpcController::write_joint_action_command(
  const vector_t& q_des, const vector_t& qd_des, const vector_t& tau_ff)
{
  // effort_pd mode: hardware evaluates kp*(q_des-q)+kd*(qd_des-qd)+tau_ff every physics
  // step (legacy CentroidalMpcMrtJointController servo). 3 command interfaces per joint.
  const size_t mpc_joint_count = parameters_.robot.jointNames.size();
  const size_t fixed_joint_count = parameters_.ocs2.model.fixedJointNames.size();
  if (command_interfaces_.size() < (mpc_joint_count + fixed_joint_count) * 3) {
    return;
  }
  auto safe = [](double v) { return std::isfinite(v) ? v : 0.0; };
  for (size_t i = 0; i < mpc_joint_count; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    command_interfaces_[3 * i + 0].set_value(safe(i < static_cast<size_t>(q_des.size()) ? q_des[idx] : 0.0));
    command_interfaces_[3 * i + 1].set_value(safe(i < static_cast<size_t>(qd_des.size()) ? qd_des[idx] : 0.0));
    command_interfaces_[3 * i + 2].set_value(safe(i < static_cast<size_t>(tau_ff.size()) ? tau_ff[idx] : 0.0));
  }
  // Joints excluded from the MPC model (wrists): hold zero, soft hardware gains.
  for (size_t i = 0; i < fixed_joint_count; ++i) {
    const size_t base = 3 * (mpc_joint_count + i);
    command_interfaces_[base + 0].set_value(0.0);
    command_interfaces_[base + 1].set_value(0.0);
    command_interfaces_[base + 2].set_value(0.0);
  }
}

}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidCentroidalMpcController,
  controller_interface::ChainableControllerInterface)
