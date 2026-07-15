#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
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

    const auto reference_config = common::buildReferenceConfig(parameters_);
    if (parameters_.ocs2.gait.gaitFile.empty()) {
      throw std::invalid_argument("[HumanoidCentroidalMpcController] ocs2.gait.gaitFile is empty.");
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
        auto target_trajectories = target_trajectories_calculator_->commandedVelocityToTargetTrajectories(
          velocity_target,
          init_time,
          init_state);
        // Centroidal state layout: yaw lives at index 9 (after the 6 momentum entries).
        heading_reference_.apply(
          velocity_target(3),
          init_time,
          control_model_->getBasePose(init_state)[3],
          target_trajectories,
          9);
        return target_trajectories;
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
      common::loadGaitMap(parameters_.ocs2.gait.gaitFile),
      reference_config,
      mpc_interface_->getSwitchedModelReferenceManagerPtr(),
      mpc_interface_->getMpcRobotModel(),
      std::move(velocity_target_to_target_trajectories),
      std::move(base_pose_target_to_target_trajectories),
      parameters_.target.defaultMode);
    motion_manager_ = std::move(motion_manager);
    rclcpp::QoS command_qos(1);
    command_qos.best_effort();
    motion_manager_->subscribe(
      get_node(), command_qos, parameters_.target.walkingVelocityTopic,
      parameters_.target.basePoseTopic, parameters_.target.modeTopic,
      parameters_.target.globalFrame);

    performance_visualization_ = std::make_unique<visualization::PerformanceVisualization>(
      get_node(),
      mpc_interface_->getPinocchioInterface(),
      mpc_interface_->getMpcRobotModel(),
      visualization::makePerformanceVisualizationSettings(parameters_));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidCentroidalMpcController] Failed to build CentroidalMpcInterface: %s",
      e.what());
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

  heading_reference_.reset();
  yaw_unwrapper_.reset();
  filtered_generalized_velocity_.resize(0);
  initial_observation_state_ = mpc_interface_->getInitialState();
  const auto initial_observation = build_observation(get_node()->now());
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

  if (parameters_.floatingBase.source == "state_interfaces") {
    if (config.type == controller_interface::interface_configuration_type::NONE) {
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    }
    const auto floating_base_interfaces = floating_base_state_interface_names();
    config.names.insert(
      config.names.end(),
      floating_base_interfaces.begin(),
      floating_base_interfaces.end());
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

ocs2::SystemObservation HumanoidCentroidalMpcController::build_observation(const rclcpp::Time& time)
{
  const auto& info = mpc_interface_->getCentroidalModelInfo();
  const auto joint_dim = static_cast<Eigen::Index>(control_model_->getJointDim());
  const auto gen_coordinates_dim = static_cast<Eigen::Index>(info.generalizedCoordinatesNum);

  ocs2::SystemObservation observation;
  observation.time = time.seconds();
  observation.state = initial_observation_state_.size() ==
    static_cast<Eigen::Index>(control_model_->getStateDim()) ?
    initial_observation_state_ :
    mpc_interface_->getInitialState();
  observation.input = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getInputDim()));
  observation.mode = ocs2::humanoid::STANCE;
  if (const auto reference_manager = mpc_interface_->getSwitchedModelReferenceManagerPtr()) {
    observation.mode = reference_manager->getModeSchedule().modeAtTime(observation.time);
  }

  // Floating base from ros2_control state interfaces (world-frame pose, body-local twist).
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

  vector_t q_joint;
  vector_t v_joint;
  if (!px || !py || !pz || !qw || !qx || !qy || !qz || !lvx || !lvy || !lvz || !avx || !avy ||
      !avz || !read_joint_state(q_joint, v_joint)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidCentroidalMpcController] missing state interfaces; holding last observation state.");
    return observation;
  }

  Eigen::Quaterniond base_orientation(*qw, *qx, *qy, *qz);
  if (base_orientation.norm() < 1e-12) {
    base_orientation = Eigen::Quaterniond::Identity();
  } else {
    base_orientation.normalize();
  }
  ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(base_orientation);
  euler_zyx(0) = yaw_unwrapper_.unwrap(euler_zyx(0));

  // Generalized coordinates and velocities in the pinocchio (centroidal) convention.
  vector_t q_pinocchio(gen_coordinates_dim);
  q_pinocchio.head<3>() = ocs2::vector3_t(*px, *py, *pz);
  q_pinocchio.segment<3>(3) = euler_zyx;
  q_pinocchio.tail(joint_dim) = q_joint;

  vector_t v_pinocchio(gen_coordinates_dim);
  v_pinocchio.head<3>() = base_orientation * ocs2::vector3_t(*lvx, *lvy, *lvz);
  v_pinocchio.segment<3>(3) =
    ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(
    euler_zyx, ocs2::vector3_t(*avx, *avy, *avz));
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
  ocs2::updateCentroidalDynamics(*control_pinocchio_, info, q_pinocchio);
  const auto& momentum_matrix = ocs2::getCentroidalMomentumMatrix(*control_pinocchio_);
  ocs2::centroidal_model::getNormalizedMomentum(observation.state, info).noalias() =
    momentum_matrix * v_pinocchio / info.robotMass;
  ocs2::centroidal_model::getGeneralizedCoordinates(observation.state, info) = q_pinocchio;

  return observation;
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
    try {
      mrt_interface_->advanceMpc();
      const auto iteration_end = Clock::now();
      advance_ms = std::chrono::duration<double, std::milli>(iteration_end - iteration_start).count();
      accumulated_advance_ms += advance_ms;
      ++advance_samples;

      if (!mrt_interface_->updatePolicy()) {
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
