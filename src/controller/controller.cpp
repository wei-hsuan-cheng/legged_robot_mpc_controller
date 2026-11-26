#include "legged_robot_mpc_controller/controller/controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

#include <legged_robot_mpc_controller/mpc/common/gait/MotionPhaseDefinition.h>
#include <legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h>
#include <legged_robot_mpc_controller/mpc/wb_id/model/WheelContactKinematics.h>

#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"
#include "legged_robot_mpc_controller/controller/config_builder.hpp"

namespace legged_robot_mpc_controller
{

namespace
{

constexpr const char* kTag = "[HumanoidWholeBodyInverseDynamicsMpcController]";

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
  return source == "state_estimator";
}

std::string estimator_report(
  const legged_robot_state_estimator::estimator_interfaces::EstimatorStatus& status,
  const legged_robot_state_estimator::state_estimation::EstimatorDiagnostics& d)
{
  char line[256];
  std::snprintf(
    line, sizeof(line),
    "epoch %u, running %.1f s, %s | roll/pitch std %.4f | height std %.4f | velocity std %.4f",
    status.epoch, status.running_time, status.ready ? "ready" : "not ready",
    d.attitude_std.head<2>().maxCoeff(), d.position_std.z(), d.velocity_std.maxCoeff());
  return line;
}

}  // namespace

HumanoidWholeBodyInverseDynamicsMpcController::~HumanoidWholeBodyInverseDynamicsMpcController()
{
  stop_solver_thread();
}

controller_interface::CallbackReturn HumanoidWholeBodyInverseDynamicsMpcController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    parameters_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s init failed: %s", kTag, e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWholeBodyInverseDynamicsMpcController::on_configure(
  const rclcpp_lifecycle::State&)
{
  if (param_listener_->is_old(parameters_)) {
    parameters_ = param_listener_->get_params();
  }
  feedforward_ = parameters_.control.feedforward == "inverse_dynamics" ?
    Feedforward::InverseDynamics : Feedforward::PolicyTorque;

  if (parameters_.paths.urdfFile.empty() || parameters_.paths.libFolder.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s paths.urdfFile or paths.libFolder is empty.", kTag);
    return controller_interface::CallbackReturn::ERROR;
  }

  // The first run generates and compiles the CppAD model libraries into
  // paths.libFolder/wb_id_mpc_<robot>, which takes a while; later runs load them.
  const char* configure_stage = "loading paths.geometryFile";
  try {
    apply_geometry_file();
    configure_stage = "building the whole-body inverse-dynamics MPC interface";
    const auto mpc_config = buildWholeBodyInverseDynamicsMpcConfig(parameters_);
    mpc_interface_ = std::make_unique<ocs2::humanoid::WbIdMpcInterface>(mpc_config);
    control_model_.reset(mpc_interface_->getMpcRobotModel().clone());
    control_pinocchio_ = std::make_unique<ocs2::PinocchioInterface>(mpc_interface_->getPinocchioInterface());
    base_to_imu_ = Eigen::Quaterniond::Identity();
    if (!parameters_.floatingBase.imuFrame.empty()) {
      const auto& model = control_pinocchio_->getModel();
      if (!model.existFrame(parameters_.floatingBase.imuFrame)) {
        throw std::invalid_argument("floatingBase.imuFrame '" + parameters_.floatingBase.imuFrame + "' is not in the URDF");
      }
      // At the neutral configuration the base is at the identity, so a frame fixed to it sits at its mounting.
      pinocchio::Data data(model);
      pinocchio::framesForwardKinematics(model, data, pinocchio::neutral(model));
      base_to_imu_ = Eigen::Quaterniond(data.oMf[model.getFrameId(parameters_.floatingBase.imuFrame)].rotation()).normalized();
    }

    const auto joint_dim = static_cast<Eigen::Index>(control_model_->getJointDim());
    torque_limit_ = mpc_interface_->getTorqueLimits();
    const bool contact_equalities = parameters_.ocs2.constraints.footMotionEquality.activate ||
                                    parameters_.ocs2.constraints.wheelContact.activate;
    if (!parameters_.ocs2.constraints.inverseDynamics.activate || !contact_equalities ||
        !parameters_.ocs2.constraints.swingFootZeroWrench.activate) {
      RCLCPP_WARN(get_node()->get_logger(),
        "%s physical dynamics/contact equalities are disabled: diagnostic ablation only, not a physically consistent locomotion model",
        kTag);
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "%s feedforward: %s | torque clamp: %s", kTag,
      feedforward_ == Feedforward::PolicyTorque && parameters_.ocs2.constraints.inverseDynamics.activate ?
        "MPC policy torque" : "inverse dynamics at the measured state",
      parameters_.ocs2.constraints.jointTorqueLimits.limits.empty() ?
        "URDF effort limits" : "ocs2.constraints.jointTorqueLimits.limits override");

    // effort_pd servo gains: written to the hardware every tick, or left to the hardware if empty.
    const auto& pd = parameters_.control.jointEffortPd;
    joint_kp_.resize(0);
    joint_kd_.resize(0);
    if (!pd.kp.empty() || !pd.kd.empty()) {
      if (static_cast<Eigen::Index>(pd.kp.size()) != joint_dim || static_cast<Eigen::Index>(pd.kd.size()) != joint_dim) {
        throw std::invalid_argument("control.jointEffortPd.kp and .kd need one entry per robot.jointNames (or both empty)");
      }
      if (parameters_.robot.commandInterface != "effort_pd") {
        throw std::invalid_argument("control.jointEffortPd needs robot.commandInterface effort_pd");
      }
      joint_kp_ = Eigen::Map<const vector_t>(pd.kp.data(), joint_dim);
      joint_kd_ = Eigen::Map<const vector_t>(pd.kd.data(), joint_dim);
    }
    if (parameters_.ocs2.constraints.wheelContact.activate) {
      // A rolling wheel's angle is unwrapped and free: a position gain would fight the rolling.
      for (const auto& wheel_joint : parameters_.ocs2.model.contactParentJointNames) {
        const auto it = std::find(parameters_.robot.jointNames.begin(), parameters_.robot.jointNames.end(), wheel_joint);
        const auto index = static_cast<Eigen::Index>(std::distance(parameters_.robot.jointNames.begin(), it));
        if (joint_kp_.size() > 0 && index < joint_kp_.size() && joint_kp_(index) != 0.0) {
          throw std::invalid_argument("control.jointEffortPd.kp of wheel joint '" + wheel_joint + "' must be 0");
        }
      }
    }
    RCLCPP_INFO(get_node()->get_logger(), "%s joint servo gains: %s", kTag,
                joint_kp_.size() > 0 ? "control.jointEffortPd, written every tick" : "the hardware's (not claimed)");

    stand_up_trajectory_.reset();
    if (!parameters_.bringUp.standUpTrajectoryFile.empty()) {
      if (joint_kp_.size() == 0) {
        throw std::invalid_argument("bringUp.standUpTrajectoryFile needs control.jointEffortPd (the playback writes servo gains)");
      }
      stand_up_trajectory_ =
        load_stand_up_trajectory(parameters_.bringUp.standUpTrajectoryFile, parameters_.robot.jointNames);
      if (!stand_up_trajectory_->kneeContacts.empty() && !parameters_.ocs2.constraints.wheelContact.activate) {
        throw std::invalid_argument("stand-up knee clearance guard needs wheelContact");
      }
      for (const auto& contact : stand_up_trajectory_->kneeContacts) {
        if (!control_pinocchio_->getModel().existFrame(contact.frame)) {
          throw std::invalid_argument("stand-up knee frame '" + contact.frame + "' is not in the URDF");
        }
      }
      check_generated_geometry("stand-up plan", parameters_.bringUp.standUpTrajectoryFile,
                               stand_up_trajectory_->wheelRadius, &*stand_up_trajectory_);
      RCLCPP_INFO(get_node()->get_logger(), "%s planned stand-up: %zu samples over %.3f s, MPC handover at %.3f s (%s)", kTag,
                  stand_up_trajectory_->time.size(), stand_up_trajectory_->time.back(), stand_up_trajectory_->handoverTime,
                  parameters_.bringUp.standUpTrajectoryFile.c_str());
    }
    sit_down_trajectory_.reset();
    if (stand_up_trajectory_ && YAML::LoadFile(parameters_.bringUp.standUpTrajectoryFile)["sit_down"]) {
      sit_down_trajectory_ =
        load_stand_up_trajectory(parameters_.bringUp.standUpTrajectoryFile, parameters_.robot.jointNames, "sit_down");
      if (sit_down_trajectory_->state.empty()) {
        throw std::invalid_argument("the planned sit-down needs its state and feedback");
      }
      check_generated_geometry("sit-down plan", parameters_.bringUp.standUpTrajectoryFile,
                               sit_down_trajectory_->wheelRadius);
      RCLCPP_INFO(get_node()->get_logger(), "%s planned sit-down: %zu samples over %.3f s from %.3f m (bringUp.sitDownHeight "
                  "unused)", kTag, sit_down_trajectory_->time.size(), sit_down_trajectory_->time.back(), sit_down_height());
    }

    configure_stage = "building the target-trajectories calculator / reference manager";
    auto reference_config = common::buildReferenceConfig(parameters_);
    reference_config.commandRateLimits =
      Eigen::Map<const ocs2::vector4_t>(parameters_.ocs2.reference.velocityCommandRateLimits.data());
    reference_config.takeoverHeightRate = parameters_.ocs2.reference.takeoverHeightRate;
    {
      std::vector<double> heights = parameters_.ocs2.reference.postureHeights;
      std::vector<double> states = parameters_.ocs2.reference.postureJointStates;
      const auto num_joints = parameters_.robot.jointNames.size();
      const auto& table_file = parameters_.ocs2.reference.postureTableFile;
      if (!table_file.empty()) {
        if (!heights.empty() || !states.empty()) {
          throw std::invalid_argument("ocs2.reference.postureTableFile replaces postureHeights/postureJointStates; set only one");
        }
        double table_wheel_radius = std::numeric_limits<double>::quiet_NaN();
        load_posture_table(table_file, parameters_.robot.jointNames, heights, states, table_wheel_radius);
        check_generated_geometry("posture table", table_file, table_wheel_radius);
        RCLCPP_INFO(get_node()->get_logger(), "%s posture table: %zu heights, %.3f-%.3f m (%s)", kTag, heights.size(),
                    heights.front(), heights.back(), table_file.c_str());
      }
      if (states.size() != heights.size() * num_joints) {
        throw std::invalid_argument("ocs2.reference.postureJointStates needs one row of robot.jointNames per postureHeights entry");
      }
      if (stand_up_trajectory_ && !heights.empty() &&
          (parameters_.bringUp.sitDownHeight < heights.front() || parameters_.bringUp.sitDownHeight > heights.back())) {
        throw std::invalid_argument("bringUp.sitDownHeight must be covered by ocs2.reference.postureHeights");
      }
      for (std::size_t i = 0; i < heights.size(); ++i) {
        reference_config.postureHeights.push_back(heights[i]);
        reference_config.postureJointStates.push_back(
          Eigen::Map<const vector_t>(states.data() + i * num_joints, static_cast<Eigen::Index>(num_joints)));
      }
    }
    target_trajectories_calculator_ = std::make_unique<ocs2::humanoid::WbIdMpcTargetTrajectoriesCalculator>(
      reference_config, mpc_interface_->getMpcRobotModel(), mpc_interface_->mpcSettings().timeHorizon_);
    auto velocity_target_to_target_trajectories =
      [this](const ocs2::vector4_t& velocity_target, ocs2::scalar_t init_time, ocs2::scalar_t /* final_time */,
             const ocs2::vector_t& init_state) {
        return target_trajectories_calculator_->commandedVelocityToTargetTrajectories(velocity_target, init_time, init_state);
      };
    auto base_pose_target_to_target_trajectories =
      [this](const ocs2::vector6_t& base_pose_target, ocs2::scalar_t init_time, ocs2::scalar_t /* final_time */,
             const ocs2::vector_t& init_state) {
        return target_trajectories_calculator_->commandedBasePoseToTargetTrajectories(base_pose_target, init_time, init_state);
      };
    // Gait templates and the velocity-based selection between them (ocs2.gait).
    auto gait_map = common::buildGaitMap(parameters_);
    auto gait_transitions = common::buildGaitTransitionConfig(parameters_, gait_map);
    auto motion_manager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      std::move(gait_map),
      std::move(gait_transitions),
      reference_config,
      mpc_interface_->getSwitchedModelReferenceManagerPtr(),
      mpc_interface_->getMpcRobotModel(),
      std::move(velocity_target_to_target_trajectories),
      std::move(base_pose_target_to_target_trajectories),
      parameters_.target.defaultMode);
    if (!parameters_.ocs2.gait.stairClimbingFile.empty()) {
      motion_manager->setStairClimbingConfig(common::loadStairClimbingConfig(parameters_.ocs2.gait.stairClimbingFile));
    }
    if (!parameters_.ocs2.gait.terrainWalkingFile.empty()) {
      mpc_interface_->getSwitchedModelReferenceManagerPtr()->setTerrainFootholdPlanner(
        std::make_shared<ocs2::humanoid::TerrainFootholdPlanner>(
          common::loadTerrainModel(parameters_.ocs2.gait.terrainWalkingFile),
          common::loadTerrainFootholdPlannerSettings(parameters_.ocs2.gait.terrainWalkingFile)));
    }
    motion_manager->setCommandFrameYaw(parameters_.target.commandFrameYaw);
    motion_manager_ = std::move(motion_manager);
    rclcpp::QoS command_qos(1);
    command_qos.best_effort();
    motion_manager_->subscribe(
      get_node(), command_qos, parameters_.target.walkingVelocityTopic,
      parameters_.target.basePoseTopic, parameters_.target.modeTopic,
      parameters_.target.globalFrame, parameters_.target.gaitTopic);
    motion_manager_->subscribeMpcTargets(
      get_node(), parameters_.target.mpcTargetsTopic,
      common::selectAtIndices(parameters_.robot.jointNames, common::findArmJointIndices(parameters_.robot.jointNames)));

    auto visualization_settings = visualization::makePerformanceVisualizationSettings(parameters_);
    auto& spheres = visualization_settings.collision_spheres;
    spheres.activate = parameters_.visualization.collisionSpheres.activate;
    spheres.marker_topic = parameters_.visualization.collisionSpheres.markerTopic;
    spheres.frame_id = parameters_.visualization.frameId;
    spheres.alpha = parameters_.visualization.collisionSpheres.alpha;
    if (spheres.activate && mpc_config.activateLegSelfCollision) {
      const auto& leg = mpc_config.costConstraintConfig.footCollisionConfig;
      for (const auto& frame : {leg.leftAnkleFrame, leg.rightAnkleFrame,
                               leg.leftFootCenterFrame, leg.rightFootCenterFrame,
                               leg.leftFootFrame1, leg.rightFootFrame1,
                               leg.leftFootFrame2, leg.rightFootFrame2}) {
        spheres.spheres.push_back({frame, frame, Eigen::Vector3d::Zero(), leg.footCollisionSphereRadius, "legs"});
      }
      for (const auto& frame : {leg.leftKneeFrame, leg.rightKneeFrame}) {
        spheres.spheres.push_back({frame, frame, Eigen::Vector3d::Zero(), leg.kneeCollisionSphereRadius, "legs"});
      }
    }
    if (spheres.activate && mpc_config.armSelfCollision.activate) {
      for (const auto& sphere : mpc_config.armSelfCollision.arms) {
        spheres.spheres.push_back({sphere.name, sphere.linkName, sphere.center, sphere.radius, "arms"});
      }
      for (const auto& sphere : mpc_config.armSelfCollision.bodies) {
        spheres.spheres.push_back({sphere.name, sphere.linkName, sphere.center, sphere.radius, "body"});
      }
    }
    performance_visualization_ = std::make_unique<visualization::PerformanceVisualization>(
      get_node(),
      mpc_interface_->getPinocchioInterface(),
      mpc_interface_->getMpcRobotModel(),
      std::move(visualization_settings));

    configure_stage = "setting up the tracking debug topic";
    configure_tracking();

    configure_stage = "connecting to the estimator controller";
    if (uses_state_estimator(parameters_.floatingBase.source) && !has_estimator()) {
      throw std::runtime_error(
        "floatingBase.source is state_estimator but floatingBase.estimator is none; launch with "
        "floating_base_estimator:=humanoid_inekf (or humanoid_linear_kf)");
    }
    estimator_values_.assign(legged_robot_state_estimator::estimator_interfaces::names().size(), 0.0);
    RCLCPP_INFO(
      get_node()->get_logger(), "%s floating base: %s | estimator controller: %s (%s)", kTag,
      parameters_.floatingBase.source.c_str(), parameters_.floatingBase.estimator.c_str(),
      !has_estimator() ? "not read" :
        uses_state_estimator(parameters_.floatingBase.source) ? "drives control" : "logged only");
    if (!uses_state_estimator(parameters_.floatingBase.source) &&
        parameters_.floatingBase.stateInterfaceName.empty()) {
      throw std::runtime_error("floatingBase.source is ground_truth_state but floatingBase.stateInterfaceName is empty");
    }
    stand_nominal_joints_ = Eigen::Map<const vector_t>(
      parameters_.ocs2.reference.defaultJointState.data(),
      static_cast<Eigen::Index>(parameters_.ocs2.reference.defaultJointState.size()));
    stand_up_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/stand_up",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const BringUpPhase phase = bring_up_phase_.load();
        if (!controller_active_.load() || phase != BringUpPhase::Passive) {
          response->success = false;
          response->message = std::string("refused: stand_up needs an active controller in idle; current phase '") +
            to_string(phase) + "'";
          return;
        }
        stand_up_requested_.store(true);
        response->success = true;
        response->message = "standing up";
      });
    start_locomotion_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/start_locomotion",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const BringUpPhase phase = bring_up_phase_.load();
        if (phase != BringUpPhase::Settle) {
          response->success = false;
          response->message = std::string("refused: start_locomotion is only valid in 'settling'; bring-up is in '") +
            to_string(phase) + "'";
          return;
        }
        if (!at_rest_.load()) {
          response->success = false;
          response->message = "refused: the trunk is not at rest yet (see the 'bring-up: settling' log); retry";
          return;
        }
        start_requested_.store(true);
        response->success = true;
        response->message = "initialising the estimator; the MPC starts once it is ready";
      });
    const auto request_posture = [this](bool toggle, auto response) {
      const auto phase = bring_up_phase_.load();
      if (!controller_active_.load() || !stand_up_trajectory_) {
        response->success = false;
        response->message = "refused: needs an active controller with a planned kneeling stand-up";
      } else if (toggle && phase == BringUpPhase::Passive) {
        stand_up_requested_.store(true);
        response->success = true;
        response->message = "stand-up requested";
      } else if (phase == BringUpPhase::Locomotion) {
        sit_down_requested_.store(true);
        response->success = true;
        response->message = "braking, then sitting down";
      } else {
        response->success = false;
        response->message = std::string("refused in '") + to_string(phase) + "'";
      }
    };
    sit_down_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/sit_down", [request_posture](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        request_posture(false, response);
      });
    toggle_stand_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/toggle_stand", [request_posture](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        request_posture(true, response);
      });
    motion_state_publisher_ = get_node()->create_publisher<std_msgs::msg::String>("~/motion_state", rclcpp::QoS(1).best_effort());
    motion_state_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(100), [this] {
      std_msgs::msg::String message;
      message.data = controller_active_.load() ? to_string(bring_up_phase_.load()) : "inactive";
      motion_state_publisher_->publish(message);
    });
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s on_configure failed while %s: %s", kTag, configure_stage, e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "%s configured | joints=%zu state_dim=%zu input_dim=%zu", kTag,
    parameters_.robot.jointNames.size(),
    static_cast<size_t>(control_model_->getStateDim()),
    static_cast<size_t>(control_model_->getInputDim()));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWholeBodyInverseDynamicsMpcController::on_activate(
  const rclcpp_lifecycle::State&)
{
  if (!mpc_interface_ || !control_model_) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s activation requested before configuration.", kTag);
    return controller_interface::CallbackReturn::ERROR;
  }

  motion_manager_->requestVelocityTargetFilterReset();
  stop_solver_thread();
  solver_observation_ready_.store(false);
  reset_mpc_warm_start_requested_.store(false);
  yaw_unwrapper_.reset();
  filtered_generalized_velocity_.resize(0);
  estimator_interface_index_.clear();
  estimator_command_index_.clear();
  gain_command_index_.clear();
  if (joint_kp_.size() > 0) {
    for (const auto& joint_name : parameters_.robot.jointNames) {
      for (const char* gain : {"kp", "kd"}) {
        const auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&](const auto& interface) {
          return interface.get_prefix_name() == joint_name && interface.get_interface_name() == gain;
        });
        if (it == command_interfaces_.end()) {
          RCLCPP_ERROR(get_node()->get_logger(), "%s servo gain interface %s/%s is not claimed.", kTag,
                       joint_name.c_str(), gain);
          return controller_interface::CallbackReturn::ERROR;
        }
        gain_command_index_.push_back(static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it)));
      }
    }
  }
  if (has_estimator()) {
    const std::string& estimator = parameters_.floatingBase.estimator;
    for (const auto& name : legged_robot_state_estimator::estimator_interfaces::names()) {
      const auto it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
          return interface.get_prefix_name() == estimator && interface.get_interface_name() == name;
        });
      if (it == state_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s estimator interface %s/%s is not claimed.", kTag,
                     estimator.c_str(), name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      estimator_interface_index_.push_back(static_cast<std::size_t>(std::distance(state_interfaces_.begin(), it)));
    }
    for (const auto& name : legged_robot_state_estimator::estimator_interfaces::reference_names()) {
      const auto it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(), [&](const auto& interface) {
          return interface.get_prefix_name() == estimator && interface.get_interface_name() == name;
        });
      if (it == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s estimator reference interface %s/%s is not claimed.", kTag,
                     estimator.c_str(), name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      estimator_command_index_.push_back(static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it)));
    }
  }
  last_estimate_ = legged_robot_state_estimator::state_estimation::FloatingBaseEstimate{};
  estimator_driving_control_ = false;

  bring_up_phase_.store(BringUpPhase::Passive);
  at_rest_.store(false);
  phase_start_time_ = -1.0;
  settled_since_ = -1.0;
  stand_up_requested_.store(false);
  start_requested_.store(false);
  sit_down_requested_.store(false);
  auto_start_pending_ = parameters_.bringUp.autoStart;
  solver_paused_.store(false);
  solver_observation_ready_.store(false);
  reset_mpc_warm_start_requested_.store(false);
  servo_kp_override_.resize(0);
  servo_kd_override_.resize(0);
  locomotion_requested_.store(false);
  mpc_ready_.store(false);
  has_active_policy_ = false;
  active_policy_ready_ = false;
  motion_manager_->setOperatorMotionEnabled(!stand_up_trajectory_);
  motion_manager_->setPostureHeightOverride(std::numeric_limits<double>::quiet_NaN());

  publish_mode_schedule();
  if (has_estimator() && !uses_state_estimator(parameters_.floatingBase.source)) {
    read_state_estimator();
    request_estimator_initialization();
  }

  if (uses_bring_up()) {
    vector_t q_joint, v_joint;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rclcpp::ok() && !read_joint_state(q_joint, v_joint)) {
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "%s joint state interfaces did not become readable within 5 s; refusing to activate.", kTag);
        return controller_interface::CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stand_up_start_joints_ = q_joint;
    passive_hold_joints_ = q_joint;
    const auto passive = damping_command(q_joint);
    write_joint_action_command(passive.policy_position, passive.policy_velocity, passive.feedforward);

    start_deferred_solver_thread();

    if (stand_up_trajectory_) {
      RCLCPP_INFO(get_node()->get_logger(),
                  "%s activated in bring-up (idle); the planned stand-up starts after %.2f s and hands over to the MPC "
                  "(%s base) at %.3f s | joints=%zu",
                  kTag, parameters_.bringUp.idleSeconds, parameters_.floatingBase.source.c_str(),
                  stand_up_trajectory_->handoverTime, parameters_.robot.jointNames.size());
    } else {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "%s activated in bring-up (passive); the MPC starts from the %s estimate once the robot has stood up "
        "and the estimator is ready | joints=%zu",
        kTag, parameters_.floatingBase.estimator.c_str(), parameters_.robot.jointNames.size());
    }
    controller_active_.store(true);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Ground-truth base: start from where the robot actually is.
  ocs2::SystemObservation initial_observation;
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rclcpp::ok()) {
      initial_observation = build_observation(get_node()->now());
      if (observation_from_hardware_) {
        break;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "%s joint and floating-base state interfaces did not become readable within 5 s; refusing to "
                     "activate.", kTag);
        return controller_interface::CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  {
    const vector_t base_pose = control_model_->getBasePose(initial_observation.state);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "%s initial state read from hardware | base xyz=(%.3f, %.3f, %.3f) rpy_zyx=(%.3f, %.3f, %.3f)", kTag,
      base_pose(0), base_pose(1), base_pose(2), base_pose(3), base_pose(4), base_pose(5));
  }
  const vector_t q_hold = control_model_->getJointAngles(initial_observation.state);
  passive_hold_joints_ = q_hold;  // held by update_and_write_commands() until the first policy
  write_joint_action_command(q_hold, vector_t::Zero(q_hold.size()), compute_weight_compensating_torque(initial_observation));

  // The solver is NOT seeded from initial_observation. That observation is stamped
  // with this node's clock, while every later one carries the controller manager's
  // update time, and the two can differ by more than the horizon at startup (the sim
  // runs while the controller activates). OCS2's MPC_BASE::run then finds the current
  // time past the end of the first horizon and refuses to solve, permanently ("The MPC
  // time-horizon is smaller than the MPC starting time"). The first control tick hands
  // over its own observation instead, as the bring-up path does; until the first
  // policy arrives update_and_write_commands() holds the measured posture.
  start_deferred_solver_thread();

  RCLCPP_INFO(get_node()->get_logger(),
              "%s activated (ground-truth base); the MPC starts from the first control tick's observation | "
              "joints=%zu",
              kTag, parameters_.robot.jointNames.size());
  controller_active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWholeBodyInverseDynamicsMpcController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  controller_active_.store(false);
  stop_solver_thread();
  // Zero the joint commands only: a 0 contact flag would tell the estimator a foot is in the
  // air, and zero servo gains would leave the joints limp, so the last gains stay in the hardware.
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (std::find(estimator_command_index_.begin(), estimator_command_index_.end(), i) ==
          estimator_command_index_.end() &&
        std::find(gain_command_index_.begin(), gain_command_index_.end(), i) == gain_command_index_.end()) {
      (void)command_interfaces_[i].set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HumanoidWholeBodyInverseDynamicsMpcController::command_interface_configuration() const
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
  // Per-joint servo gains after the joint commands, so their [3 * i + k] layout is unchanged.
  if (effort_pd && !parameters_.control.jointEffortPd.kp.empty()) {
    for (const auto& joint_name : parameters_.robot.jointNames) {
      config.names.emplace_back(joint_name + "/kp");
      config.names.emplace_back(joint_name + "/kd");
    }
  }
  if (!config.names.empty()) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  }
  if (has_estimator()) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& name : legged_robot_state_estimator::estimator_interfaces::reference_names()) {
      config.names.emplace_back(parameters_.floatingBase.estimator + "/" + name);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
HumanoidWholeBodyInverseDynamicsMpcController::state_interface_configuration() const
{
  auto config = make_joint_interface_configuration(parameters_.robot.stateInterfaces);
  for (const auto& joint_name : parameters_.ocs2.model.fixedJointNames) {
    for (const auto& interface_name : parameters_.robot.stateInterfaces) {
      if (!interface_name.empty()) {
        config.names.emplace_back(joint_name + "/" + interface_name);
      }
    }
  }

  const bool need_gt_interfaces =
    parameters_.floatingBase.source == "ground_truth_state" ||
    !parameters_.floatingBase.stateInterfaceName.empty();
  const bool bring_up = uses_bring_up();
  if ((need_gt_interfaces || has_estimator() || bring_up) &&
      config.type == controller_interface::interface_configuration_type::NONE) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  }
  if (need_gt_interfaces) {
    const auto floating_base_interfaces = floating_base_state_interface_names();
    config.names.insert(config.names.end(), floating_base_interfaces.begin(), floating_base_interfaces.end());
  }
  if (has_estimator()) {
    for (const auto& name : legged_robot_state_estimator::estimator_interfaces::names()) {
      config.names.emplace_back(parameters_.floatingBase.estimator + "/" + name);
    }
  }
  if (parameters_.diagnostics.trackingRate > 0.0 && !parameters_.diagnostics.contactSensorNames.empty()) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& sensor : parameters_.diagnostics.contactSensorNames) {
      config.names.emplace_back(sensor + "/normal_force");
      config.names.emplace_back(sensor + "/contact");
    }
  }
  if (bring_up) {
    const std::string& imu = parameters_.floatingBase.imuInterfaceName;
    for (const char* axis : {"x", "y", "z"}) {
      config.names.emplace_back(imu + "/angular_velocity." + axis);
      config.names.emplace_back(imu + "/linear_acceleration." + axis);
    }
    for (const char* component : {"w", "x", "y", "z"}) {
      config.names.emplace_back(imu + "/orientation." + component);
    }
  }
  return config;
}

controller_interface::return_type HumanoidWholeBodyInverseDynamicsMpcController::update_reference_from_subscribers(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type HumanoidWholeBodyInverseDynamicsMpcController::update_and_write_commands(
  const rclcpp::Time& time, const rclcpp::Duration&)
{
  if (!mpc_interface_ || !control_model_) {
    return controller_interface::return_type::ERROR;
  }

  read_state_estimator();

  const auto observation = build_observation(time);
  const bool solver_ready = mpc_ready_.load(std::memory_order_acquire);
  if (solver_ready && observation_from_hardware_ && std::isfinite(observation.time) && observation.state.allFinite()) {
    // As in the arm MPCs, feed observations even while holding for an initial policy.
    const double delay = parameters_.ocs2.mpc.testObservationDelay;
    if (delay > 0.0) {
      // Robustness tests: plan from an observation `delay` old, like a slower computer.
      delayed_observations_.push_back(observation);
      while (delayed_observations_.size() > 1 && delayed_observations_[1].time <= observation.time - delay) {
        delayed_observations_.pop_front();
      }
      mrt_interface_->setCurrentObservation(delayed_observations_.front());
    } else {
      mrt_interface_->setCurrentObservation(observation);
    }
    solver_observation_ready_.store(true, std::memory_order_release);
  }
  // The worker only fills the MRT buffer. Swap the active policy here so evaluation
  // and visualization cannot read a policy being replaced by the solver thread.
  const bool policy_updated = solver_ready && locomotion_requested_.load() && mrt_interface_->updatePolicy();
  const bool previous_policy_ready = active_policy_ready_;
  if (policy_updated) {
    has_active_policy_ = true;
  }
  active_policy_ready_ = false;
  if (has_active_policy_) {
    const auto& times = mrt_interface_->getPolicy().timeTrajectory_;
    const double evaluation_time = observation.time + parameters_.ocs2.solver.rollout.policyTimeOffset;
    const double margin = 2.0 / std::max(1.0, static_cast<double>(get_update_rate()));
    active_policy_ready_ = observation_from_hardware_ && times.size() >= 2 &&
      std::isfinite(evaluation_time) && std::isfinite(times.front()) && std::isfinite(times.back()) &&
      evaluation_time >= times.front() && evaluation_time + margin < times.back();
    if (!active_policy_ready_) {
      // Match dynamics_mpc_controller: request reset here, execute it on the worker.
      // Request once per rejected plan, not every hold tick while a new solve runs.
      if (policy_updated || previous_policy_ready) {
        reset_mpc_warm_start_requested_.store(true, std::memory_order_release);
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "%s rejecting unusable policy: evaluation=%.6f, plan=[%.6f, %.6f]; holding and requesting a new horizon.",
        kTag, evaluation_time, times.empty() ? 0.0 : times.front(), times.empty() ? 0.0 : times.back());
    }
  }
  JointActionCommand command;
  if (uses_bring_up()) {
    command = run_bring_up(time, observation);
  } else if (active_policy_ready_) {
    command = compute_mpc_joint_action(observation);
    passive_hold_joints_ = command.policy_position;
  } else {
    // Ground-truth base, first ticks: seed the solver from this tick's observation
    // (see on_activate) and hold the activation pose until the first policy.
    if (!locomotion_requested_.load(std::memory_order_acquire) && observation_from_hardware_) {
      start_observation_ = observation;
      locomotion_requested_.store(true, std::memory_order_release);
    }
    // Hold the pose measured at activation, not this tick's: with q_des = q the joint
    // servo has nothing to correct and the robot sags under gravity while the solver
    // is built.
    command.policy_position = passive_hold_joints_.size() > 0 ? passive_hold_joints_
                                                              : control_model_->getJointAngles(observation.state);
    command.policy_velocity = vector_t::Zero(command.policy_position.size());
    command.feedforward = compute_weight_compensating_torque(observation);
  }

  write_joint_action_command(command.policy_position, command.policy_velocity, command.feedforward);
  write_estimator_references(time.seconds());
  if (tracking_publisher_ && ++tracking_tick_ >= tracking_decimation_) {
    tracking_tick_ = 0;
    publish_tracking(observation, command);
  }
  tracking_policy_valid_ = false;

  if (performance_visualization_ && active_policy_ready_ &&
      (last_visualization_time_ < 0.0 || observation.time - last_visualization_time_ >= 0.1)) {
    last_visualization_time_ = observation.time;
    const auto& policy = mrt_interface_->getPolicy();
    performance_visualization_->update_visualization(
      policy.timeTrajectory_, policy.stateTrajectory_, mrt_interface_->getCommand().mpcTargetTrajectories_,
      observation.state);
  }

  return controller_interface::return_type::OK;
}

const char* HumanoidWholeBodyInverseDynamicsMpcController::to_string(BringUpPhase phase)
{
  switch (phase) {
    case BringUpPhase::Passive:
      return "idle";
    case BringUpPhase::StandUp:
      return "standing up";
    case BringUpPhase::Settle:
      return "settling";
    case BringUpPhase::Converge:
      return "estimator converging";
    case BringUpPhase::StartingMpc:
      return "starting MPC";
    case BringUpPhase::Blend:
      return "blending into MPC";
    case BringUpPhase::Locomotion:
      return "locomotion";
    case BringUpPhase::Stopping:
      return "stopping";
    case BringUpPhase::Lowering:
      return "lowering";
    case BringUpPhase::SitDown:
      return "sitting down";
    case BringUpPhase::Fallen:
      return "fallen";
    case BringUpPhase::Fault:
      return "FAULT (damping)";
  }
  return "unknown";
}

std::optional<Eigen::Quaterniond> HumanoidWholeBodyInverseDynamicsMpcController::read_bring_up_orientation()
{
  // Ground-truth bring-up must check trunk tilt, not the possibly rotated IMU site.
  const std::string& frame = uses_state_estimator(parameters_.floatingBase.source)
                               ? parameters_.floatingBase.imuInterfaceName
                               : parameters_.floatingBase.stateInterfaceName;
  const auto qw = get_state_interface_value(frame, "orientation.w");
  const auto qx = get_state_interface_value(frame, "orientation.x");
  const auto qy = get_state_interface_value(frame, "orientation.y");
  const auto qz = get_state_interface_value(frame, "orientation.z");
  if (!qw || !qx || !qy || !qz) {
    return std::nullopt;
  }
  const Eigen::Quaterniond q(*qw, *qx, *qy, *qz);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) {
    return std::nullopt;
  }
  // The IMU attitude is the sensor's; the trunk's is R_world_imu * R_base_imu^T.
  return uses_state_estimator(parameters_.floatingBase.source) ? (q.normalized() * base_to_imu_.conjugate()).normalized()
                                                               : q.normalized();
}

bool HumanoidWholeBodyInverseDynamicsMpcController::read_imu_rates(Eigen::Vector3d& gyro, Eigen::Vector3d& accel)
{
  const std::string& imu = parameters_.floatingBase.imuInterfaceName;
  const char* axes[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    const auto g = get_state_interface_value(imu, std::string("angular_velocity.") + axes[i]);
    const auto a = get_state_interface_value(imu, std::string("linear_acceleration.") + axes[i]);
    if (!g || !a) {
      return false;
    }
    gyro(i) = *g;
    accel(i) = *a;
  }
  return gyro.allFinite() && accel.allFinite();
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::damping_command(const vector_t& q_measured) const
{
  JointActionCommand command;
  command.policy_position = q_measured;
  command.policy_velocity = vector_t::Zero(q_measured.size());
  command.feedforward = vector_t::Zero(q_measured.size());
  return command;
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::hold_command(
  const vector_t& q_target, const vector_t& q_measured,
  const std::optional<Eigen::Quaterniond>& imu_orientation)
{
  JointActionCommand command;
  command.policy_position = q_target;
  command.policy_velocity = vector_t::Zero(q_target.size());
  command.feedforward = vector_t::Zero(q_target.size());
  if (!imu_orientation) {
    return command;
  }
  // Gravity compensation from the trunk attitude and the joints only - no base
  // position, no estimator - so the stand works before the estimator exists.
  ocs2::SystemObservation stand;
  stand.state = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getStateDim()));
  stand.input = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getInputDim()));
  vector_t generalized(6 + q_measured.size());
  generalized.head<3>().setZero();
  generalized.segment<3>(3) = ocs2::humanoid::quaternionToEulerZYX(*imu_orientation);
  generalized.tail(q_measured.size()) = q_measured;
  control_model_->setGeneralizedCoordinates(stand.state, generalized);
  command.feedforward = compute_weight_compensating_torque(stand);
  return command;
}

std::array<std::string, 2> HumanoidWholeBodyInverseDynamicsMpcController::contact_frame_names() const
{
  const auto& names = mpc_interface_->modelSettings().contactNames;
  return {names.at(0), names.at(1)};
}

bool HumanoidWholeBodyInverseDynamicsMpcController::uses_bring_up() const
{
  return uses_state_estimator(parameters_.floatingBase.source) || stand_up_trajectory_.has_value();
}

HumanoidWholeBodyInverseDynamicsMpcController::WheeledBipedGeometry
HumanoidWholeBodyInverseDynamicsMpcController::load_geometry(const std::string& file)
{
  const YAML::Node root = YAML::LoadFile(file)["geometry"];
  if (!root || !root["wheel"]) {
    throw std::invalid_argument("geometry " + file + " has no 'geometry.wheel' map");
  }
  const auto vector3 = [&](const YAML::Node& node, const std::string& what) {
    const auto values = node.as<std::vector<double>>();
    if (values.size() != 3 || !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) {
      throw std::invalid_argument("geometry " + file + ": " + what + " needs three finite values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  };
  const YAML::Node wheel = root["wheel"];
  WheeledBipedGeometry geometry;
  geometry.wheelRadius = wheel["radius"].as<double>();
  geometry.wheelHalfWidth = wheel["half_width"].as<double>();
  geometry.wheelCenterOffset = vector3(wheel["center_offset"], "wheel.center_offset");
  geometry.wheelAxleAxis = vector3(wheel["axle_axis"], "wheel.axle_axis");
  if (!(geometry.wheelRadius > 0.0 && geometry.wheelHalfWidth >= 0.0) ||
      !std::isfinite(geometry.wheelRadius + geometry.wheelHalfWidth) || geometry.wheelAxleAxis.norm() < 1e-9) {
    throw std::invalid_argument("geometry " + file + ": wheel needs radius > 0, half_width >= 0 and a nonzero axle_axis");
  }
  const YAML::Node housings = root["knee_housings"];
  for (std::size_t i = 0; housings && i < housings.size(); ++i) {
    const YAML::Node housing = housings[i];
    StandUpTrajectory::KneeContact knee;
    knee.frame = housing["frame"].as<std::string>();
    knee.center = vector3(housing["center"], "knee_housings center");
    knee.radius = housing["radius"].as<double>();
    knee.halfWidth = housing["half_width"].as<double>();
    if (knee.frame.empty() || !(knee.radius > 0.0 && knee.halfWidth >= 0.0) || !std::isfinite(knee.radius + knee.halfWidth)) {
      throw std::invalid_argument("geometry " + file + ": invalid knee_housings entry");
    }
    geometry.kneeHousings.push_back(std::move(knee));
  }
  return geometry;
}

void HumanoidWholeBodyInverseDynamicsMpcController::apply_geometry_file()
{
  geometry_.reset();
  const auto& file = parameters_.paths.geometryFile;
  if (file.empty()) {
    return;
  }
  // The file is the single source: a second value in the parameter file could silently disagree with it.
  const auto overrides = get_node()->get_node_parameters_interface()->get_parameter_overrides();
  for (const char* name : {"ocs2.constraints.wheelContact.radius", "ocs2.constraints.wheelContact.axleAxis",
                           "ocs2.model.contactGeometry.frameTranslation"}) {
    if (overrides.count(name) > 0) {
      throw std::invalid_argument(std::string(name) + " comes from paths.geometryFile; remove it from the parameter file");
    }
  }
  geometry_ = load_geometry(file);
  auto& wheel = parameters_.ocs2.constraints.wheelContact;
  wheel.radius = geometry_->wheelRadius;
  wheel.axleAxis = {geometry_->wheelAxleAxis.x(), geometry_->wheelAxleAxis.y(), geometry_->wheelAxleAxis.z()};
  const auto& offset = geometry_->wheelCenterOffset;
  parameters_.ocs2.model.contactGeometry.frameTranslation = {offset.x(), offset.y(), offset.z()};
  RCLCPP_INFO(get_node()->get_logger(),
              "%s geometry: wheel radius %.4f m, half width %.4f m, tire centre [%.4f %.4f %.4f], %zu knee housings (%s)",
              kTag, geometry_->wheelRadius, geometry_->wheelHalfWidth, offset.x(), offset.y(), offset.z(),
              geometry_->kneeHousings.size(), file.c_str());
}

void HumanoidWholeBodyInverseDynamicsMpcController::check_generated_geometry(
  const std::string& what, const std::string& file, double wheel_radius, const StandUpTrajectory* plan) const
{
  if (!geometry_) {
    return;
  }
  const auto same = [](double a, double b) { return std::abs(a - b) <= 1e-9; };
  const std::string regenerate = what + " " + file + " was generated for other geometry than paths.geometryFile (" +
                                 parameters_.paths.geometryFile + "); regenerate it (config/upkie/regenerate.sh): ";
  if (!std::isfinite(wheel_radius)) {
    throw std::invalid_argument(regenerate + "it records no wheel_radius");
  }
  if (!same(wheel_radius, geometry_->wheelRadius)) {
    throw std::invalid_argument(regenerate + "wheel radius " + std::to_string(wheel_radius));
  }
  if (plan == nullptr || plan->kneeContacts.empty()) {
    return;
  }
  bool match = same(plan->wheelHalfWidth, geometry_->wheelHalfWidth) &&
               plan->kneeContacts.size() == geometry_->kneeHousings.size();
  for (std::size_t i = 0; match && i < plan->kneeContacts.size(); ++i) {
    const auto& a = plan->kneeContacts[i];
    const auto& b = geometry_->kneeHousings[i];
    match = a.frame == b.frame && (a.center - b.center).cwiseAbs().maxCoeff() <= 1e-9 && same(a.radius, b.radius) &&
            same(a.halfWidth, b.halfWidth);
  }
  if (!match) {
    throw std::invalid_argument(regenerate + "wheel half width or knee housings differ");
  }
}

void HumanoidWholeBodyInverseDynamicsMpcController::load_posture_table(
  const std::string& file, const std::vector<std::string>& joint_names, std::vector<double>& heights,
  std::vector<double>& joint_states, double& wheel_radius)
{
  const YAML::Node root = YAML::LoadFile(file)["posture_table"];
  if (!root) {
    throw std::invalid_argument("posture table " + file + " has no 'posture_table' map");
  }
  wheel_radius = root["wheel_radius"] ? root["wheel_radius"].as<double>() : std::numeric_limits<double>::quiet_NaN();
  if (root["joint_names"].as<std::vector<std::string>>() != joint_names) {
    throw std::invalid_argument("posture table " + file + ": joint_names must equal robot.jointNames, in order");
  }
  heights = root["heights"].as<std::vector<double>>();
  joint_states.clear();
  const auto rows = root["joint_states"].as<std::vector<std::vector<double>>>();
  if (heights.empty() || rows.size() != heights.size()) {
    throw std::invalid_argument("posture table " + file + ": joint_states needs one row per height, with >= 1 height");
  }
  for (const auto& row : rows) {
    if (row.size() != joint_names.size()) {
      throw std::invalid_argument("posture table " + file + ": every joint_states row needs one value per joint");
    }
    joint_states.insert(joint_states.end(), row.begin(), row.end());
  }
  if (!std::all_of(heights.begin(), heights.end(), [](double h) { return std::isfinite(h); }) ||
      !std::all_of(joint_states.begin(), joint_states.end(), [](double q) { return std::isfinite(q); })) {
    throw std::invalid_argument("posture table " + file + ": heights and joint_states must be finite");
  }
}

HumanoidWholeBodyInverseDynamicsMpcController::StandUpTrajectory
HumanoidWholeBodyInverseDynamicsMpcController::load_stand_up_trajectory(
  const std::string& file, const std::vector<std::string>& joint_names, const std::string& key)
{
  const YAML::Node root = YAML::LoadFile(file)[key];
  if (!root) {
    throw std::invalid_argument("stand-up trajectory " + file + " has no '" + key + "' map");
  }
  if (root["joint_names"].as<std::vector<std::string>>() != joint_names) {
    throw std::invalid_argument("stand-up trajectory " + file + ": joint_names must equal robot.jointNames, in order");
  }
  const auto n = static_cast<Eigen::Index>(joint_names.size());
  const auto to_vector = [&](const std::vector<double>& values, const std::string& what, Eigen::Index count = 0) {
    if (count == 0) count = n;
    if (static_cast<Eigen::Index>(values.size()) != count) {
      throw std::invalid_argument("stand-up trajectory " + file + ": wrong row size for " + what);
    }
    vector_t result = Eigen::Map<const vector_t>(values.data(), count);
    if (!result.allFinite()) {
      throw std::invalid_argument("stand-up trajectory " + file + ": " + what + " must be finite");
    }
    return result;
  };
  StandUpTrajectory plan;
  plan.time = root["time"].as<std::vector<double>>();
  plan.handoverTime = root["handover_time"].as<double>();
  plan.mpcStartTime = root["mpc_start_time"].as<double>(0.0);
  plan.kp = to_vector(root["kp"].as<std::vector<double>>(), "kp");
  plan.kd = to_vector(root["kd"].as<std::vector<double>>(), "kd");
  for (const char* key : {"position", "velocity", "effort"}) {
    auto& samples = std::string(key) == "position" ? plan.position : (std::string(key) == "velocity" ? plan.velocity : plan.effort);
    for (const auto& row : root[key].as<std::vector<std::vector<double>>>()) {
      samples.push_back(to_vector(row, key));
    }
    if (samples.size() != plan.time.size()) {
      throw std::invalid_argument("stand-up trajectory " + file + ": " + key + " needs one row per time sample");
    }
  }
  const bool finiteTimes = std::all_of(plan.time.begin(), plan.time.end(), [](double t) { return std::isfinite(t); });
  const bool strictlyIncreasing = std::adjacent_find(plan.time.begin(), plan.time.end(),
                                                    [](double a, double b) { return a >= b; }) == plan.time.end();
  if (plan.time.size() < 2 || !finiteTimes || !strictlyIncreasing || plan.time.front() != 0.0) {
    throw std::invalid_argument("stand-up trajectory " + file + ": time must start at 0 and increase, with >= 2 samples");
  }
  if (!(plan.handoverTime > 0.0 && plan.handoverTime <= plan.time.back())) {
    throw std::invalid_argument("stand-up trajectory " + file + ": handover_time must lie inside the trajectory");
  }
  if (!(plan.mpcStartTime >= 0.0 && plan.mpcStartTime < plan.handoverTime)) {
    throw std::invalid_argument("stand-up trajectory " + file + ": mpc_start_time must precede handover_time");
  }
  if (root["state"] || root["feedback"]) {
    const Eigen::Index state_dim = 2 * (6 + n);
    for (const auto& row : root["state"].as<std::vector<std::vector<double>>>()) {
      plan.state.push_back(to_vector(row, "state", state_dim));
    }
    for (const auto& row : root["feedback"].as<std::vector<std::vector<double>>>()) {
      const vector_t flat = to_vector(row, "feedback", n * state_dim);
      plan.feedback.emplace_back(Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        flat.data(), n, state_dim));
    }
    if (plan.state.size() != plan.time.size() || plan.feedback.size() != plan.time.size()) {
      throw std::invalid_argument("stand-up trajectory " + file + ": state/feedback need one row per time sample");
    }
    for (std::size_t i = 0; i < plan.time.size(); ++i) {
      if (!plan.state[i].segment(6, n).isApprox(plan.position[i], 1e-7) ||
          !plan.state[i].tail(n).isApprox(plan.velocity[i], 1e-7)) {
        throw std::invalid_argument("stand-up trajectory " + file + ": state and joint reference disagree");
      }
    }
  }
  if (root["wheel_radius"]) {
    plan.wheelRadius = root["wheel_radius"].as<double>();
  }
  if (root["knee_contacts"]) {
    plan.minimumKneeClearance = root["minimum_knee_clearance"].as<double>();
    plan.clearanceHoldTime = root["clearance_hold_time"].as<double>();
    plan.wheelHalfWidth = root["wheel_half_width"].as<double>();
    if (!(plan.minimumKneeClearance > 0.0 && plan.clearanceHoldTime > 0.0 && plan.wheelHalfWidth >= 0.0) ||
        !std::isfinite(plan.minimumKneeClearance + plan.clearanceHoldTime + plan.wheelHalfWidth)) {
      throw std::invalid_argument("stand-up trajectory " + file + ": invalid support guard");
    }
    for (const auto& contact : root["knee_contacts"]) {
      StandUpTrajectory::KneeContact knee;
      knee.frame = contact["frame"].as<std::string>();
      knee.center = to_vector(contact["center"].as<std::vector<double>>(), "knee center", 3);
      knee.radius = contact["radius"].as<double>();
      knee.halfWidth = contact["half_width"].as<double>();
      if (knee.frame.empty() || !(knee.radius > 0.0 && knee.halfWidth >= 0.0) ||
          !std::isfinite(knee.radius + knee.halfWidth)) {
        throw std::invalid_argument("stand-up trajectory " + file + ": invalid knee geometry");
      }
      plan.kneeContacts.push_back(std::move(knee));
    }
    if (plan.kneeContacts.empty() || plan.feedback.empty()) {
      throw std::invalid_argument("stand-up trajectory " + file + ": support guard needs contacts and feedback");
    }
  }
  return plan;
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::StandUpTrajectory::sample(double t, const vector_t& measured_state) const
{
  // Linear interpolation, held at the ends.
  const auto upper = std::upper_bound(time.begin(), time.end(), t);
  const size_t k1 = static_cast<size_t>(std::clamp<std::ptrdiff_t>(std::distance(time.begin(), upper), 1,
                                                                    static_cast<std::ptrdiff_t>(time.size()) - 1));
  const size_t k0 = k1 - 1;
  const double a = std::clamp((t - time[k0]) / (time[k1] - time[k0]), 0.0, 1.0);
  JointActionCommand command;
  command.policy_position = (1.0 - a) * position[k0] + a * position[k1];
  command.policy_velocity = (1.0 - a) * velocity[k0] + a * velocity[k1];
  command.feedforward = (1.0 - a) * effort[k0] + a * effort[k1];
  if (!feedback.empty() && measured_state.size() > 0) {
    const vector_t error = measured_state - ((1.0 - a) * state[k0] + a * state[k1]);
    command.feedforward -= ((1.0 - a) * feedback[k0] + a * feedback[k1]) * error;
  }
  return command;
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::sample_stand_up(double t, const ocs2::SystemObservation& observation) const
{
  return sample_plan(*stand_up_trajectory_, stand_up_anchor_state_, t, observation);
}

double HumanoidWholeBodyInverseDynamicsMpcController::sit_down_height() const
{
  return sit_down_trajectory_ ? sit_down_trajectory_->state.front()(2) : parameters_.bringUp.sitDownHeight;
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::sample_plan(const StandUpTrajectory& plan, const vector_t& anchor, double t,
                                                           const ocs2::SystemObservation& observation) const
{
  if (plan.feedback.empty()) return plan.sample(t);
  vector_t state = observation.state;
  const auto& initial = plan.state.front();
  const auto n = static_cast<Eigen::Index>(plan.kp.size());
  const double yaw = anchor(3) - initial(3);
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d delta = state.head<3>() - anchor.head<3>();
  delta.z() = 0.0;
  const Eigen::Vector3d position = initial.head<3>() + rotation * delta;
  state.head<2>() = position.head<2>();
  state.segment<3>(6 + n) = rotation * state.segment<3>(6 + n).eval();
  state(3) = initial(3) + std::atan2(std::sin(state(3) - anchor(3)),
                                  std::cos(state(3) - anchor(3)));
  vector_t offsets = vector_t::Zero(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    if (plan.kp(i) == 0.0) offsets(i) = anchor(6 + i) - initial(6 + i);
  }
  state.segment(6, n) -= offsets;
  auto command = plan.sample(t, state);
  command.policy_position += offsets;
  return command;
}

bool HumanoidWholeBodyInverseDynamicsMpcController::stand_up_support_ready(const ocs2::SystemObservation& observation)
{
  const auto& plan = *stand_up_trajectory_;
  if (plan.kneeContacts.empty()) return true;
  const auto& model = control_pinocchio_->getModel();
  auto& data = control_pinocchio_->getData();
  pinocchio::forwardKinematics(model, data, control_model_->getGeneralizedCoordinates(observation.state));
  const auto lowest = [](const Eigen::Vector3d& center, const Eigen::Vector3d& axle, double radius, double width) {
    const double az = std::clamp(axle.z(), -1.0, 1.0);
    return center.z() - radius * std::sqrt(std::max(0.0, 1.0 - az * az)) - width * std::abs(az);
  };
  const auto& wheel = parameters_.ocs2.constraints.wheelContact;
  const Eigen::Vector3d axle(wheel.axleAxis[0], wheel.axleAxis[1], wheel.axleAxis[2]);
  std::array<double, 2> ground;
  const auto frames = contact_frame_names();
  for (std::size_t i = 0; i < ground.size(); ++i) {
    const auto& frame = pinocchio::updateFramePlacement(model, data, model.getFrameId(frames[i]));
    ground[i] = lowest(frame.translation(), frame.rotation() * axle.normalized(), wheel.radius, plan.wheelHalfWidth);
  }
  if (std::abs(ground[0] - ground[1]) > 0.01) return false;
  const double plane = 0.5 * (ground[0] + ground[1]);
  for (const auto& knee : plan.kneeContacts) {
    if (!model.existFrame(knee.frame)) return false;
    const auto& frame = pinocchio::updateFramePlacement(model, data, model.getFrameId(knee.frame));
    if (lowest(frame.act(knee.center), frame.rotation().col(2), knee.radius, knee.halfWidth) - plane <
        plan.minimumKneeClearance) return false;
  }
  return true;
}

void HumanoidWholeBodyInverseDynamicsMpcController::enter_bring_up_phase(BringUpPhase phase, double time)
{
  RCLCPP_INFO(get_node()->get_logger(), "%s bring-up: %s -> %s", kTag, to_string(bring_up_phase_.load()),
              to_string(phase));
  bring_up_phase_.store(phase);
  at_rest_.store(false);
  phase_start_time_ = time;
  settled_since_ = -1.0;
  rest_filter_initialized_ = false;
  if (phase == BringUpPhase::Fallen || phase == BringUpPhase::Fault) {
    locomotion_requested_.store(false, std::memory_order_release);
    stand_up_requested_.store(false);
    start_requested_.store(false);
    sit_down_requested_.store(false);
    has_active_policy_ = active_policy_ready_ = false;
    motion_manager_->setOperatorMotionEnabled(false);
    servo_kp_override_ = vector_t::Zero(joint_kp_.size());
    servo_kd_override_ = joint_kd_;
  }
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::run_bring_up(
  const rclcpp::Time& time, const ocs2::SystemObservation& observation)
{
  const double t = time.seconds();
  const auto& bu = parameters_.bringUp;
  vector_t q, v;
  if (!read_joint_state(q, v)) {
    if (stand_up_trajectory_ && bring_up_phase_ != BringUpPhase::Fault && bring_up_phase_ != BringUpPhase::Fallen) {
      enter_bring_up_phase(BringUpPhase::Fault, t);
    }
    return damping_command(stand_nominal_joints_);
  }
  const auto orientation = read_bring_up_orientation();
  if (phase_start_time_ < 0.0) {
    phase_start_time_ = t;
  }

  if (!orientation && stand_up_trajectory_ && bring_up_phase_ != BringUpPhase::Fault && bring_up_phase_ != BringUpPhase::Fallen) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s no valid trunk attitude; latching FAULT.", kTag);
    enter_bring_up_phase(BringUpPhase::Fault, t);
  }
  if (orientation && (bring_up_phase_ != BringUpPhase::Passive || stand_up_trajectory_) && bring_up_phase_ != BringUpPhase::Fault &&
      bring_up_phase_ != BringUpPhase::Fallen) {
    const double tilt = std::acos(std::clamp(orientation->toRotationMatrix()(2, 2), -1.0, 1.0));
    if (tilt > bu.tiltLimit) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%s bring-up FAULT: trunk tilt %.3f rad > %.3f in phase '%s'; damping. Deactivate and "
                   "reactivate the controller to retry.",
                   kTag, tilt, bu.tiltLimit, to_string(bring_up_phase_.load()));
      enter_bring_up_phase(BringUpPhase::Fallen, t);
    }
  }

  if (sit_down_requested_.exchange(false) && bring_up_phase_ == BringUpPhase::Locomotion) {
    motion_manager_->setOperatorMotionEnabled(false);
    enter_bring_up_phase(BringUpPhase::Stopping, t);
  }

  switch (bring_up_phase_.load()) {
    case BringUpPhase::Passive: {
      JointActionCommand hold = damping_command(passive_hold_joints_);
      if (stand_up_trajectory_) {
        // Wheeled: initialize the estimator in supported idle and execute the tracked stand-up.
        // Contact-aware plans defer wheel-only solves until sustained knee clearance; legacy plans
        // can start them immediately. The idle hold uses the plan's servo gains, so the motion
        // starts from the supported state used by the offline model.
        servo_kp_override_ = stand_up_trajectory_->kp;
        servo_kd_override_ = stand_up_trajectory_->kd;
        if (!(auto_start_pending_ || stand_up_requested_.load()) || t - phase_start_time_ < bu.idleSeconds) {
          return hold;
        }
        const double idle_error = ((q - stand_up_trajectory_->position.front()).array() *
          (stand_up_trajectory_->kp.array() > 0.0).cast<double>()).abs().maxCoeff();
        if (idle_error > 0.2 || v.norm() > 0.5) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                              "%s stand-up waiting for the supported idle pose (joint error %.3f rad).", kTag, idle_error);
          return hold;
        }
        // A previous cycle may still be finishing its last solve. Reset MRT only
        // after the worker acknowledges the pause, never join it on the control thread.
        if (mpc_ready_.load(std::memory_order_acquire)) {
          if (!solver_paused_.load(std::memory_order_acquire)) {
            return hold;
          }
          mrt_interface_->reset();
          has_active_policy_ = false;
          active_policy_ready_ = false;
          reset_mpc_warm_start_requested_.store(true, std::memory_order_release);
        }
        if (uses_state_estimator(parameters_.floatingBase.source)) {
          if (!estimator_initialized_for_request()) {
            request_estimator_initialization();
            return hold;
          }
          if (!estimator_status_.ready) {
            RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                 "%s bring-up: idle, estimator converging | %s", kTag,
                                 estimator_report(estimator_status_, estimator_diagnostics_).c_str());
            return hold;
          }
          estimator_driving_control_ = true;
        }
        if (!observation_from_hardware_) {
          return hold;
        }
        start_observation_ = observation;
        if (mpc_ready_.load(std::memory_order_acquire)) {
          mrt_interface_->setCurrentObservation(observation);
        }
        motion_manager_->requestVelocityTargetFilterReset(control_model_->getBasePose(observation.state)(2));
        motion_manager_->setPostureHeightOverride(std::numeric_limits<double>::quiet_NaN());
        auto_start_pending_ = false;
        stand_up_requested_.store(false);
        locomotion_requested_.store(stand_up_trajectory_->mpcStartTime <= 0.0, std::memory_order_release);
        stand_up_anchor_state_ = observation.state;
        stand_up_support_since_ = -1.0;
        stand_up_start_joints_ = q;
        enter_bring_up_phase(BringUpPhase::StandUp, t);
        return hold;
      }
      if (bu.autoStart || stand_up_requested_.load()) {
        stand_up_start_joints_ = q;
        enter_bring_up_phase(BringUpPhase::StandUp, t);
      } else {
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                             "%s bring-up: passive (holding the activation pose); call ~/stand_up with the feet on "
                             "the floor.", kTag);
      }
      return hold;
    }

    case BringUpPhase::StandUp: {
      if (stand_up_trajectory_) {
        const auto& plan = *stand_up_trajectory_;
        const double elapsed = t - phase_start_time_;
        if (!plan.feedback.empty() && (!observation_from_hardware_ || !observation.state.allFinite())) {
          enter_bring_up_phase(BringUpPhase::Fault, t);
          servo_kp_override_.resize(0);
          servo_kd_override_.resize(0);
          return damping_command(q);
        }
        servo_kp_override_ = plan.kp;
        servo_kd_override_ = plan.kd;
        const bool support_ready = stand_up_support_ready(observation);
        stand_up_support_since_ = support_ready ? (stand_up_support_since_ < 0.0 ? t : stand_up_support_since_) : -1.0;
        const bool support_held = stand_up_support_since_ >= 0.0 &&
          t - stand_up_support_since_ >= plan.clearanceHoldTime;
        if (!locomotion_requested_.load(std::memory_order_acquire) && elapsed >= plan.mpcStartTime && support_held) {
          // Do not seed a wheel-only OCP with a knee-supported configuration.
          start_observation_ = observation;
          motion_manager_->setPostureHeightOverride(control_model_->getBasePose(observation.state)(2));
          motion_manager_->requestVelocityTargetFilterReset(control_model_->getBasePose(observation.state)(2));
          locomotion_requested_.store(true, std::memory_order_release);
          RCLCPP_INFO(get_node()->get_logger(), "%s stand-up: knees clear; starting wheel-only MPC at %.3f s.", kTag, elapsed);
        }
        if (elapsed >= plan.handoverTime) {
          if (!support_held || !active_policy_ready_) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "%s bring-up FAULT: stand-up handover rejected at %.3f s (support geometry %s, MPC policy %s); damping.",
                         kTag, plan.handoverTime, support_held ? "ready" : "not ready", active_policy_ready_ ? "ready" : "not ready");
            servo_kp_override_.resize(0);
            servo_kd_override_.resize(0);
            enter_bring_up_phase(BringUpPhase::Fault, t);
            return damping_command(q);
          }
          // The height reference restarts at the measured height and ramps to the command at the
          // configured height rate, instead of stepping from the crouch to the standing height.
          motion_manager_->requestVelocityTargetFilterReset(control_model_->getBasePose(observation.state)(2));
          motion_manager_->setPostureHeightOverride(std::numeric_limits<double>::quiet_NaN());
          enter_bring_up_phase(BringUpPhase::Blend, t);
          JointActionCommand handover = sample_stand_up(plan.handoverTime, observation);
          clamp_torque(handover.feedforward);
          return handover;
        }
        // The plan's feed-forward is clamped like the MPC's (jointTorqueLimits / URDF efforts).
        JointActionCommand playback = sample_stand_up(elapsed, observation);
        clamp_torque(playback.feedforward);
        return playback;
      }
      const double s = bu.standUpSeconds > 0.0
        ? std::clamp((t - phase_start_time_) / bu.standUpSeconds, 0.0, 1.0) : 1.0;
      const double alpha = s * s * (3.0 - 2.0 * s);
      const vector_t target = (1.0 - alpha) * stand_up_start_joints_ + alpha * stand_nominal_joints_;
      if (s >= 1.0) {
        enter_bring_up_phase(BringUpPhase::Settle, t);
      }
      return hold_command(target, q, orientation);
    }

    case BringUpPhase::Settle: {
      Eigen::Vector3d gyro, accel;
      bool rest = false;
      if (read_imu_rates(gyro, accel)) {
        const double dt = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));
        const double a = 1.0 - std::exp(-2.0 * M_PI * bu.settleFilterHz * dt);
        if (!rest_filter_initialized_) {
          rest_gyro_lpf_ = gyro;
          rest_accel_lpf_ = accel;
          rest_filter_initialized_ = true;
        }
        rest_gyro_lpf_ += a * (gyro - rest_gyro_lpf_);
        rest_accel_lpf_ += a * (accel - rest_accel_lpf_);
        rest = rest_gyro_lpf_.norm() < bu.settleGyroMax &&
               std::abs(rest_accel_lpf_.norm() - 9.81) < bu.settleAccelTolerance;
      }
      settled_since_ = rest ? (settled_since_ < 0.0 ? t : settled_since_) : -1.0;
      const bool settled = settled_since_ >= 0.0 && t - settled_since_ >= bu.settleSeconds;
      at_rest_.store(settled);
      if (settled && (bu.autoStart || start_requested_.load())) {
        request_estimator_initialization();
        enter_bring_up_phase(BringUpPhase::Converge, t);
      } else {
        RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "%s bring-up: settling | gyro %.3f/%.2f rad/s | |accel|-g %+.3f/%.2f m/s^2 | %s", kTag,
          rest_gyro_lpf_.norm(), bu.settleGyroMax, rest_accel_lpf_.norm() - 9.81, bu.settleAccelTolerance,
          settled ? "at rest; call ~/start_locomotion" : "not at rest yet");
      }
      return hold_command(stand_nominal_joints_, q, orientation);
    }

    case BringUpPhase::Converge:
      if (estimator_initialized_for_request() && estimator_status_.ready && observation_from_hardware_) {
        start_observation_ = observation;
        locomotion_requested_.store(true, std::memory_order_release);
        RCLCPP_INFO(get_node()->get_logger(), "%s bring-up: estimator %s ready (%s); starting the MPC from its estimate.",
                    kTag, parameters_.floatingBase.estimator.c_str(),
                    estimator_report(estimator_status_, estimator_diagnostics_).c_str());
        enter_bring_up_phase(BringUpPhase::StartingMpc, t);
      } else {
        RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s bring-up: estimator converging | %s", kTag,
          estimator_initialized_for_request() ?
            estimator_report(estimator_status_, estimator_diagnostics_).c_str() :
            "waiting for the estimator to act on the initialization request");
      }
      return hold_command(stand_nominal_joints_, q, orientation);

    case BringUpPhase::StartingMpc:
      if (active_policy_ready_) {
        estimator_driving_control_ = true;
        enter_bring_up_phase(BringUpPhase::Blend, t);
      }
      return hold_command(stand_nominal_joints_, q, orientation);

    case BringUpPhase::Blend: {
      if (!active_policy_ready_) {
        estimator_driving_control_ = false;
        enter_bring_up_phase(BringUpPhase::StartingMpc, t);
        return hold_command(stand_nominal_joints_, q, orientation);
      }
      const JointActionCommand stand = stand_up_trajectory_ ? sample_stand_up(stand_up_trajectory_->handoverTime, observation)
                                                            : hold_command(stand_nominal_joints_, q, orientation);
      const JointActionCommand mpc = compute_mpc_joint_action(observation);
      const double a = bu.blendSeconds > 0.0
        ? std::clamp((t - phase_start_time_) / bu.blendSeconds, 0.0, 1.0) : 1.0;
      JointActionCommand blended = mpc;
      blended.policy_position = (1.0 - a) * stand.policy_position + a * mpc.policy_position;
      blended.policy_velocity = (1.0 - a) * stand.policy_velocity + a * mpc.policy_velocity;
      blended.feedforward = (1.0 - a) * stand.feedforward + a * mpc.feedforward;
      if (stand_up_trajectory_) {
        servo_kp_override_ = (1.0 - a) * stand_up_trajectory_->kp + a * joint_kp_;
        servo_kd_override_ = (1.0 - a) * stand_up_trajectory_->kd + a * joint_kd_;
      }
      if (a >= 1.0) {
        servo_kp_override_.resize(0);
        servo_kd_override_.resize(0);
        if (stand_up_trajectory_) {
          motion_manager_->setOperatorMotionEnabled(true);
        }
        enter_bring_up_phase(BringUpPhase::Locomotion, t);
      }
      return blended;
    }

    case BringUpPhase::Locomotion:
      if (!active_policy_ready_) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s no valid MPC policy during locomotion; latching FAULT.", kTag);
        estimator_driving_control_ = false;
        enter_bring_up_phase(BringUpPhase::Fault, t);
        return damping_command(q);
      }
      return compute_mpc_joint_action(observation);

    case BringUpPhase::Stopping: {
      if (!active_policy_ready_) {
        enter_bring_up_phase(BringUpPhase::Fault, t);
        break;
      }
      const auto velocity = control_model_->getBaseComVelocity(observation.state);
      if (t - phase_start_time_ >= bu.sitDownBrakeSeconds &&
          velocity.head<2>().norm() < bu.sitDownVelocityMax && velocity.tail<3>().norm() < 0.3) {
        motion_manager_->requestVelocityTargetFilterReset(control_model_->getBasePose(observation.state)(2));
        motion_manager_->setPostureHeightOverride(sit_down_height());
        enter_bring_up_phase(BringUpPhase::Lowering, t);
      } else if (t - phase_start_time_ > bu.sitDownBrakeSeconds + 5.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s sit-down could not brake to rest; latching FAULT.", kTag);
        enter_bring_up_phase(BringUpPhase::Fault, t);
        break;
      } else {
        return compute_mpc_joint_action(observation);
      }
      return compute_mpc_joint_action(observation);
    }
    case BringUpPhase::Lowering: {
      if (!active_policy_ready_) {
        enter_bring_up_phase(BringUpPhase::Fault, t);
        break;
      }
      const auto base = control_model_->getBasePose(observation.state);
      bool crouched = base(2) <= bu.sitDownHeight + 0.02 && orientation &&
                      orientation->toRotationMatrix()(2, 2) > std::cos(0.3);
      if (sit_down_trajectory_) {
        // The planned sit-down starts from rest at its first state: height, level trunk and leg posture.
        const auto& plan = *sit_down_trajectory_;
        const auto velocity = control_model_->getBaseComVelocity(observation.state);
        const double posture_error = ((q - plan.position.front()).array() *
          (plan.kp.array() > 0.0).cast<double>()).abs().maxCoeff();
        crouched = std::abs(base(2) - sit_down_height()) < 0.01 && orientation &&
                   orientation->toRotationMatrix()(2, 2) > std::cos(0.1) && velocity.head<2>().norm() < 0.05 &&
                   velocity.tail<3>().norm() < 0.2 && posture_error < 0.1;
        if (crouched) {
          sit_down_anchor_state_ = observation.state;
        }
      }
      if (crouched) {
        stand_up_start_joints_ = q;
        locomotion_requested_.store(false, std::memory_order_release);
        has_active_policy_ = active_policy_ready_ = false;
        enter_bring_up_phase(BringUpPhase::SitDown, t);
      } else if (t - phase_start_time_ > 8.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s sit-down could not reach its balanced crouch; latching FAULT.", kTag);
        enter_bring_up_phase(BringUpPhase::Fault, t);
        break;
      } else {
        return compute_mpc_joint_action(observation);
      }
      [[fallthrough]];
    }
    case BringUpPhase::SitDown: {
      if (sit_down_trajectory_) {
        const auto& plan = *sit_down_trajectory_;
        servo_kp_override_ = plan.kp;
        servo_kd_override_ = plan.kd;
        const double elapsed = t - phase_start_time_;
        JointActionCommand command = sample_plan(plan, sit_down_anchor_state_, elapsed, observation);
        clamp_torque(command.feedforward);
        if (elapsed >= plan.time.back() + 0.5 && v.norm() < 1.0) {
          stand_up_requested_.store(false);
          estimator_driving_control_ = false;
          enter_bring_up_phase(BringUpPhase::Passive, t);
        } else if (elapsed > plan.time.back() + 5.0) {
          RCLCPP_ERROR(get_node()->get_logger(), "%s planned sit-down did not come to rest; latching FAULT.", kTag);
          enter_bring_up_phase(BringUpPhase::Fault, t);
        }
        return command;
      }
      servo_kp_override_ = stand_up_trajectory_->kp;
      servo_kd_override_ = stand_up_trajectory_->kd;
      const double s = std::clamp((t - phase_start_time_) / bu.sitDownSeconds, 0.0, 1.0);
      const double alpha = s * s * (3.0 - 2.0 * s);
      // Never drive an unwrapped wheel angle back to the plan's original spin angle.
      // Return to the same servo target used before the first stand-up. Holding
      // the plan's settled measured angles with zero feedforward would sag again.
      const vector_t delta = (passive_hold_joints_ - stand_up_start_joints_).array() *
        (stand_up_trajectory_->kp.array() > 0.0).cast<double>();
      const vector_t target = stand_up_start_joints_ + alpha * delta;
      auto command = damping_command(target);
      command.policy_velocity = (6.0 * s * (1.0 - s) / bu.sitDownSeconds) * delta;
      // Wheel angles are unwrapped and free, so arrival checks only position-controlled joints.
      const double error = ((q - target).array() * (stand_up_trajectory_->kp.array() > 0.0).cast<double>()).abs().maxCoeff();
      if (s >= 1.0 && error < 0.15 && v.norm() < 1.0) {
        stand_up_requested_.store(false);
        estimator_driving_control_ = false;
        enter_bring_up_phase(BringUpPhase::Passive, t);
      } else if (t - phase_start_time_ > bu.sitDownSeconds + 5.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s sit-down did not reach the kneeling pose; latching FAULT.", kTag);
        enter_bring_up_phase(BringUpPhase::Fault, t);
      }
      return command;
    }

    case BringUpPhase::Fallen:
    case BringUpPhase::Fault:
      break;
  }
  locomotion_requested_.store(false, std::memory_order_release);
  has_active_policy_ = active_policy_ready_ = false;
  motion_manager_->setOperatorMotionEnabled(false);
  servo_kp_override_ = vector_t::Zero(q.size());
  servo_kd_override_ = joint_kd_;
  return damping_command(q);
}

std::vector<hardware_interface::CommandInterface>
HumanoidWholeBodyInverseDynamicsMpcController::on_export_reference_interfaces()
{
  // controller_manager rejects chainable controllers with zero reference interfaces.
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.emplace_back(
    std::string(get_node()->get_name()),
    std::string("dummy_humanoid_whole_body_inverse_dynamics_mpc/") + hardware_interface::HW_IF_EFFORT,
    reference_interfaces_.data());
  return reference_interfaces;
}

controller_interface::InterfaceConfiguration
HumanoidWholeBodyInverseDynamicsMpcController::make_joint_interface_configuration(
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

std::vector<std::string> HumanoidWholeBodyInverseDynamicsMpcController::floating_base_state_interface_names() const
{
  const std::string& name = parameters_.floatingBase.stateInterfaceName;
  return {
    name + "/position.x", name + "/position.y", name + "/position.z",
    name + "/orientation.w", name + "/orientation.x", name + "/orientation.y", name + "/orientation.z",
    name + "/linear_velocity.x", name + "/linear_velocity.y", name + "/linear_velocity.z",
    name + "/angular_velocity.x", name + "/angular_velocity.y", name + "/angular_velocity.z",
  };
}

std::optional<double> HumanoidWholeBodyInverseDynamicsMpcController::get_state_interface_value(
  const std::string& prefix_name, const std::string& interface_name) const
{
  const auto state_handle = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [&prefix_name, &interface_name](const auto& interface) {
      return interface.get_prefix_name() == prefix_name && interface.get_interface_name() == interface_name;
    });
  if (state_handle == state_interfaces_.end()) {
    return std::nullopt;
  }
  return state_handle->get_optional();
}

bool HumanoidWholeBodyInverseDynamicsMpcController::read_joint_state(vector_t& q, vector_t& v)
{
  const size_t n = parameters_.robot.jointNames.size();
  const auto position_offset = interface_offset(parameters_.robot.stateInterfaces, hardware_interface::HW_IF_POSITION);
  const auto velocity_offset = interface_offset(parameters_.robot.stateInterfaces, hardware_interface::HW_IF_VELOCITY);
  if (!position_offset || !velocity_offset) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                          "%s stateInterfaces must include position and velocity.", kTag);
    return false;
  }
  const size_t stride = parameters_.robot.stateInterfaces.size();
  if (state_interfaces_.size() < n * stride) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                          "%s expected at least %zu state interfaces, got %zu.", kTag, n * stride,
                          state_interfaces_.size());
    return false;
  }
  q.resize(static_cast<Eigen::Index>(n));
  v.resize(static_cast<Eigen::Index>(n));
  for (size_t i = 0; i < n; ++i) {
    const auto qi = state_interfaces_[i * stride + *position_offset].get_optional();
    const auto vi = state_interfaces_[i * stride + *velocity_offset].get_optional();
    if (!qi || !vi) {
      return false;
    }
    q[static_cast<Eigen::Index>(i)] = *qi;
    v[static_cast<Eigen::Index>(i)] = *vi;
  }
  return true;
}

void HumanoidWholeBodyInverseDynamicsMpcController::publish_mode_schedule()
{
  // Solver thread only, between iterations.
  if (const auto reference_manager = mpc_interface_->getSwitchedModelReferenceManagerPtr()) {
    mode_schedule_buffer_.writeFromNonRT(reference_manager->getModeSchedule());
  }
}

ocs2::contact_flag_t HumanoidWholeBodyInverseDynamicsMpcController::contact_flags_at(double time)
{
  return ocs2::humanoid::modeNumber2StanceLeg(mode_at(time));
}

size_t HumanoidWholeBodyInverseDynamicsMpcController::mode_at(double time)
{
  const auto* schedule = mode_schedule_buffer_.readFromRT();
  if (schedule == nullptr || schedule->modeSequence.empty()) {
    return ocs2::humanoid::STANCE;
  }
  return schedule->modeAtTime(time);
}

void HumanoidWholeBodyInverseDynamicsMpcController::read_state_estimator()
{
  if (estimator_interface_index_.empty()) {
    return;
  }
  for (std::size_t i = 0; i < estimator_interface_index_.size(); ++i) {
    const auto value = state_interfaces_[estimator_interface_index_[i]].get_optional();
    if (!value) {
      return;
    }
    estimator_values_[i] = *value;
  }
  legged_robot_state_estimator::estimator_interfaces::unpack(estimator_values_, last_estimate_, estimator_diagnostics_, estimator_status_);
  last_estimate_.valid = last_estimate_.valid && estimator_initialized_for_request();
}

void HumanoidWholeBodyInverseDynamicsMpcController::request_estimator_initialization()
{
  estimator_epoch_ = estimator_status_.epoch + 1;
  last_estimate_.valid = false;
}

void HumanoidWholeBodyInverseDynamicsMpcController::write_estimator_references(double time)
{
  if (estimator_command_index_.size() != legged_robot_state_estimator::estimator_interfaces::reference_names().size()) {
    return;
  }
  const auto phase = bring_up_phase_.load();
  const bool mpc_driving = !uses_state_estimator(parameters_.floatingBase.source) ||
    phase == BringUpPhase::Blend || phase == BringUpPhase::Locomotion;
  const auto contact = mpc_driving ? contact_flags_at(time) : ocs2::contact_flag_t{true, true};
  auto& left = command_interfaces_[estimator_command_index_[legged_robot_state_estimator::estimator_interfaces::kContactLeft]];
  auto& right = command_interfaces_[estimator_command_index_[legged_robot_state_estimator::estimator_interfaces::kContactRight]];
  auto& epoch = command_interfaces_[estimator_command_index_[legged_robot_state_estimator::estimator_interfaces::kInitializeEpoch]];
  (void)left.set_value(contact[0] ? 1.0 : 0.0);
  (void)right.set_value(contact[1] ? 1.0 : 0.0);
  (void)epoch.set_value(static_cast<double>(estimator_epoch_));
}

bool HumanoidWholeBodyInverseDynamicsMpcController::estimator_initialized_for_request() const
{
  return estimator_epoch_ != 0 && estimator_status_.epoch == estimator_epoch_;
}

ocs2::SystemObservation HumanoidWholeBodyInverseDynamicsMpcController::build_observation(const rclcpp::Time& time)
{
  const auto joint_dim = static_cast<Eigen::Index>(control_model_->getJointDim());
  const auto gen_coordinates_dim = static_cast<Eigen::Index>(control_model_->getGenCoordinatesDim());

  observation_from_hardware_ = false;

  ocs2::SystemObservation observation;
  observation.time = time.seconds();
  observation.state = has_hardware_observation_ ? last_hardware_observation_.state : mpc_interface_->getInitialState();
  observation.input = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getInputDim()));
  observation.mode = mode_at(observation.time);

  vector_t q_joint;
  vector_t v_joint;
  if (!read_joint_state(q_joint, v_joint)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                          "%s missing joint state interfaces; holding last observation.", kTag);
    return hold_last_hardware_observation(observation);
  }

  ocs2::vector3_t base_position;
  Eigen::Quaterniond base_orientation;
  ocs2::vector3_t linear_velocity_world;
  ocs2::vector3_t angular_velocity_local;

  if (uses_state_estimator(parameters_.floatingBase.source)) {
    if (!last_estimate_.valid) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "%s no valid floating-base estimate yet; holding last observation.", kTag);
      return hold_last_hardware_observation(observation);
    }
    base_position = last_estimate_.position;
    base_orientation = last_estimate_.orientation;
    linear_velocity_world = last_estimate_.linear_velocity_world;
    angular_velocity_local = last_estimate_.angular_velocity_local;
  } else {
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
    if (!px || !py || !pz || !qw || !qx || !qy || !qz || !lvx || !lvy || !lvz || !avx || !avy || !avz) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                            "%s missing floating-base interfaces; holding last observation.", kTag);
      return hold_last_hardware_observation(observation);
    }
    base_position = ocs2::vector3_t(*px, *py, *pz);
    base_orientation = Eigen::Quaterniond(*qw, *qx, *qy, *qz);
    // The GT twist is body-local; rotate the linear part into the world frame.
    const Eigen::Quaterniond gt_orientation =
      base_orientation.norm() < 1e-12 ? Eigen::Quaterniond::Identity() : base_orientation.normalized();
    linear_velocity_world = gt_orientation * ocs2::vector3_t(*lvx, *lvy, *lvz);
    angular_velocity_local = ocs2::vector3_t(*avx, *avy, *avz);
  }

  if (base_orientation.norm() < 1e-12) {
    base_orientation = Eigen::Quaterniond::Identity();
  } else {
    base_orientation.normalize();
  }
  ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(base_orientation);
  euler_zyx(0) = yaw_unwrapper_.unwrap(euler_zyx(0));

  vector_t q_pinocchio(gen_coordinates_dim);
  q_pinocchio.head<3>() = base_position;
  q_pinocchio.segment<3>(3) = euler_zyx;
  q_pinocchio.tail(joint_dim) = q_joint;

  vector_t v_pinocchio(gen_coordinates_dim);
  v_pinocchio.head<3>() = linear_velocity_world;
  v_pinocchio.segment<3>(3) =
    ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(euler_zyx, angular_velocity_local);
  v_pinocchio.tail(joint_dim) = v_joint;

  // Low-pass the observation velocity: raw simulator velocity
  // dither at node 0 otherwise keeps the SQP's initial defect above g_max.
  const double cutoff_hz = parameters_.control.observationVelocityFilterCutoffHz;
  if (cutoff_hz > 0.0) {
    const double dt = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt);
    if (filtered_generalized_velocity_.size() != v_pinocchio.size()) {
      filtered_generalized_velocity_ = v_pinocchio;
    } else {
      filtered_generalized_velocity_ = alpha * v_pinocchio + (1.0 - alpha) * filtered_generalized_velocity_;
    }
    v_pinocchio = filtered_generalized_velocity_;
  }

  // Whole-body state: [q, v] in the pinocchio convention, straight from the
  // floating-base source and the joint interfaces.
  control_model_->setGeneralizedCoordinates(observation.state, q_pinocchio);
  control_model_->setGeneralizedVelocities(observation.state, v_pinocchio);

  observation_from_hardware_ = true;
  last_hardware_observation_ = observation;
  has_hardware_observation_ = true;
  return observation;
}

ocs2::SystemObservation HumanoidWholeBodyInverseDynamicsMpcController::hold_last_hardware_observation(
  const ocs2::SystemObservation& fallback) const
{
  if (!has_hardware_observation_) {
    return fallback;
  }
  ocs2::SystemObservation held = last_hardware_observation_;
  held.time = fallback.time;
  held.mode = fallback.mode;
  return held;
}

ocs2::TargetTrajectories HumanoidWholeBodyInverseDynamicsMpcController::current_observation_to_reset_trajectory(
  const ocs2::SystemObservation& observation)
{
  // Hold the current pose upright and at rest: zero pitch/roll and zero velocities.
  vector_t target_state = observation.state;
  target_state.segment<2>(4).setZero();
  control_model_->setGeneralizedVelocities(
    target_state, vector_t::Zero(static_cast<Eigen::Index>(control_model_->getGenCoordinatesDim())));
  return ocs2::TargetTrajectories({observation.time}, {target_state}, {vector_t::Zero(observation.input.size())});
}

void HumanoidWholeBodyInverseDynamicsMpcController::start_deferred_solver_thread()
{
  // Solver construction and resets stay on the worker. Observations keep arriving
  // while the control loop holds the standing posture and waits for a fresh policy.
  stop_solver_thread();
  terminate_solver_thread_.store(false);
  solver_thread_ = std::jthread([this] {
    while (!terminate_solver_thread_.load() && rclcpp::ok() &&
           !locomotion_requested_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (terminate_solver_thread_.load() || !rclcpp::ok()) {
      return;
    }
    const auto construction_start = std::chrono::steady_clock::now();
    try {
      construct_solver(start_observation_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s failed to start the MPC solver: %s", kTag, e.what());
      return;
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "%s solver initialized in %.3f s; waiting for a fresh policy before takeover.",
      kTag, std::chrono::duration<double>(std::chrono::steady_clock::now() - construction_start).count());
    mpc_ready_.store(true, std::memory_order_release);
    // Construction may have taken longer than a horizon. Do not solve the old
    // seed: the next control tick supplies the current observation through MRT.
    while (!terminate_solver_thread_.load() && rclcpp::ok() &&
           !solver_observation_ready_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    solver_worker();
  });
}

void HumanoidWholeBodyInverseDynamicsMpcController::construct_solver(const ocs2::SystemObservation& initial_observation)
{
  const std::string& solver_type = parameters_.ocs2.mpc.solverType;
  if (solver_type != "sqp") {
    throw std::runtime_error(std::string(kTag) + " unsupported solver type: " + solver_type);
  }
  mpc_solver_ = std::make_unique<ocs2::SqpMpc>(
    mpc_interface_->mpcSettings(), mpc_interface_->sqpSettings(), mpc_interface_->getOptimalControlProblem(),
    mpc_interface_->getInitializer());
  mpc_solver_->getSolverPtr()->setReferenceManager(mpc_interface_->getReferenceManagerPtr());
  mrt_interface_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_solver_);
  mpc_solver_->getSolverPtr()->addSynchronizedModule(motion_manager_);
  mpc_solver_->getSolverPtr()->addSynchronizedModule(mpc_interface_->getTorqueWindowModule());
  mrt_interface_->setCurrentObservation(initial_observation);
  mrt_interface_->resetMpcNode(current_observation_to_reset_trajectory(initial_observation));
}

void HumanoidWholeBodyInverseDynamicsMpcController::stop_solver_thread()
{
  terminate_solver_thread_.store(true);
  if (solver_thread_.joinable()) {
    solver_thread_.join();
  }
  mpc_ready_.store(false);
  locomotion_requested_.store(false);
  mrt_interface_.reset();
  mpc_solver_.reset();
}

void HumanoidWholeBodyInverseDynamicsMpcController::solver_worker()
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
  // Solver health over the log period: does the returned plan actually satisfy the
  // dynamics and the equality constraints (RNEA, contact), and how slow is the worst
  // solve? A converged-but-infeasible plan and a falling robot look alike otherwise.
  double max_advance_ms = 0.0;
  double max_equality_sse = 0.0;
  double max_dynamics_sse = 0.0;
  double accumulated_equality_sse = 0.0;
  std::size_t solver_exceptions = 0;
  // The same violation, per equality term (RNEA rows, stance/swing feet, zero wrench),
  // worst solve over the log period: which constraint the plan fails to satisfy.
  std::vector<std::pair<std::string, std::size_t>> equality_terms;
  for (const auto& [name, index] : mpc_interface_->getOptimalControlProblem().equalityConstraintPtr->getTermNameMap()) {
    equality_terms.emplace_back(name, index);
  }
  std::vector<double> max_term_sse(equality_terms.size(), 0.0);
  bool first_solve = true;

  while (!terminate_solver_thread_.load() && rclcpp::ok()) {
    if (!locomotion_requested_.load(std::memory_order_acquire)) {
      solver_paused_.store(true, std::memory_order_release);
      previous_start = Clock::time_point{};
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    solver_paused_.store(false, std::memory_order_release);
    const auto iteration_start = Clock::now();
    const auto wakeup_time = iteration_start + period;
    if (previous_start != Clock::time_point{}) {
      accumulated_period_ms += std::chrono::duration<double, std::milli>(iteration_start - previous_start).count();
      ++period_samples;
    }
    previous_start = iteration_start;

    double advance_ms = 0.0;
    try {
      if (reset_mpc_warm_start_requested_.exchange(false, std::memory_order_acq_rel)) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "%s resetting MPC warm start after an unusable policy; next solve uses the current observation and target.",
          kTag);
        mrt_interface_->resetMpcNode(mpc_interface_->getReferenceManagerPtr()->getTargetTrajectories());
      }
      mrt_interface_->advanceMpc();
      advance_ms = std::chrono::duration<double, std::milli>(Clock::now() - iteration_start).count();
      if (first_solve) {
        RCLCPP_INFO(
          get_node()->get_logger(), "%s first MPC advance: horizon end=%.6f, wall time=%.3f ms.",
          kTag, mpc_solver_->getSolverPtr()->getFinalTime(), advance_ms);
        first_solve = false;
      }
      accumulated_advance_ms += advance_ms;
      ++advance_samples;
      max_advance_ms = std::max(max_advance_ms, advance_ms);
      publish_mode_schedule();
      if (solver_publisher_) {
        publish_solver_metrics(advance_ms);
      }
      // Between iterations, so the solver is idle and this read does not race.
      const auto& performance = mpc_solver_->getSolverPtr()->getPerformanceIndeces();
      max_equality_sse = std::max(max_equality_sse, performance.equalityConstraintsSSE);
      max_dynamics_sse = std::max(max_dynamics_sse, performance.dynamicsViolationSSE);
      accumulated_equality_sse += performance.equalityConstraintsSSE;
      const auto& metrics = mpc_solver_->getSolverPtr()->getSolutionMetrics();
      for (std::size_t t = 0; t < equality_terms.size(); ++t) {
        double sse = 0.0;
        for (const auto& node : metrics.intermediates) {
          if (equality_terms[t].second < node.stateInputEqConstraint.size()) {
            sse += node.stateInputEqConstraint[equality_terms[t].second].squaredNorm();
          }
        }
        max_term_sse[t] = std::max(max_term_sse[t], sse);
      }
    } catch (const std::exception& e) {
      reset_mpc_warm_start_requested_.store(true, std::memory_order_release);
      ++solver_exceptions;
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s MPC solver exception: %s",
                            kTag, e.what());
    }

    if (Clock::now() >= next_log && advance_samples > 0) {
      const double average_period_ms = period_samples > 0 ?
        accumulated_period_ms / static_cast<double>(period_samples) : target_period_ms;
      const double average_advance_ms = accumulated_advance_ms / static_cast<double>(advance_samples);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "%s[MPC_TIMING] target=%.2f Hz, actual=%.2f Hz, latest advance=%.3f ms, average advance=%.3f ms, "
        "max advance=%.3f ms, target period=%.3f ms, utilization=%.1f%%.",
        kTag, target_frequency, 1000.0 / average_period_ms, advance_ms, average_advance_ms, max_advance_ms,
        target_period_ms, 100.0 * average_advance_ms / target_period_ms);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "%s[MPC_HEALTH] equality SSE mean=%.3e max=%.3e | dynamics SSE max=%.3e | solver exceptions=%zu | "
        "torque-window overruns=%lu",
        kTag, accumulated_equality_sse / static_cast<double>(advance_samples), max_equality_sse, max_dynamics_sse,
        solver_exceptions, static_cast<unsigned long>(torque_window_overruns_.load()));
      {
        std::vector<std::size_t> order(equality_terms.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return max_term_sse[a] > max_term_sse[b]; });
        std::string worst;
        for (std::size_t k = 0; k < std::min<std::size_t>(3, order.size()); ++k) {
          char entry[160];
          std::snprintf(entry, sizeof(entry), "%s%s=%.2e", k > 0 ? ", " : "", equality_terms[order[k]].first.c_str(),
                        max_term_sse[order[k]]);
          worst += entry;
        }
        RCLCPP_INFO(get_node()->get_logger(), "%s[MPC_HEALTH] worst equality terms (max SSE): %s", kTag, worst.c_str());
        std::fill(max_term_sse.begin(), max_term_sse.end(), 0.0);
      }
      accumulated_period_ms = 0.0;
      accumulated_advance_ms = 0.0;
      period_samples = 0;
      advance_samples = 0;
      max_advance_ms = 0.0;
      max_equality_sse = 0.0;
      max_dynamics_sse = 0.0;
      accumulated_equality_sse = 0.0;
      solver_exceptions = 0;
      next_log = Clock::now() + log_period;
    }

    std::this_thread::sleep_until(wakeup_time);
  }
}

HumanoidWholeBodyInverseDynamicsMpcController::vector_t
HumanoidWholeBodyInverseDynamicsMpcController::compute_weight_compensating_torque(
  const ocs2::SystemObservation& observation)
{
  const vector_t input = ocs2::humanoid::weightCompensatingInput(*control_pinocchio_, {true, true}, *control_model_);
  const std::array<ocs2::vector6_t, 2> foot_wrenches{
    control_model_->getContactWrench(input, 0), control_model_->getContactWrench(input, 1)};
  const vector_t q = control_model_->getGeneralizedCoordinates(observation.state);
  const vector_t qd = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getGenCoordinatesDim()));
  const vector_t qdd_joints = vector_t::Zero(static_cast<Eigen::Index>(control_model_->getJointDim()));
  return ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(q, qd, qdd_joints, foot_wrenches, *control_pinocchio_,
                                                            contact_frame_names());
}

void HumanoidWholeBodyInverseDynamicsMpcController::clamp_torque(vector_t& tau) const
{
  if (torque_limit_.size() == tau.size()) {
    tau = tau.cwiseMax(-torque_limit_).cwiseMin(torque_limit_);
  }
}

HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand
HumanoidWholeBodyInverseDynamicsMpcController::compute_mpc_joint_action(const ocs2::SystemObservation& observation)
{
  JointActionCommand command;
  vector_t policy_state;
  vector_t policy_input;
  size_t policy_mode = ocs2::humanoid::STANCE;
  const double policy_time = observation.time + parameters_.ocs2.solver.rollout.policyTimeOffset;
  mrt_interface_->evaluatePolicy(policy_time, observation.state, policy_state, policy_input, policy_mode);
  if (tracking_publisher_) {
    tracking_policy_valid_ = true;
    tracking_policy_state_ = policy_state;
    tracking_policy_input_ = policy_input;
    tracking_policy_mode_ = policy_mode;
  }

  // Joint servo targets: the planned joint positions and velocities.
  command.policy_position = control_model_->getJointAngles(policy_state);
  command.policy_velocity = control_model_->getJointVelocities(policy_state, policy_input);

  const auto plan_inverse_dynamics = [&](const vector_t& q, const vector_t& v) {
    // RNEA with the planned joint accelerations and wrenches, the base acceleration
    // from the floating-base equations of motion.
    const std::array<ocs2::vector6_t, 2> foot_wrenches{
      control_model_->getContactWrench(policy_input, 0), control_model_->getContactWrench(policy_input, 1)};
    const vector_t a = control_model_->getGeneralizedAccelerations(policy_input);
    vector_t tau = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
      q, v, a.tail(static_cast<Eigen::Index>(control_model_->getJointDim())), foot_wrenches, *control_pinocchio_,
      contact_frame_names());
    // The modelled joint Coulomb friction, as in the MPC's joint dynamics rows.
    const auto& friction = mpc_interface_->modelSettings().jointFriction;
    const auto nj = static_cast<Eigen::Index>(friction.size());
    if (nj == tau.size()) {
      const vector_t joint_velocities = v.tail(nj);
      for (Eigen::Index i = 0; i < nj; ++i) {
        tau(i) += friction[static_cast<size_t>(i)] *
                  std::tanh(joint_velocities(i) / mpc_interface_->modelSettings().jointFrictionVelocity);
      }
    }
    return tau;
  };

  if (feedforward_ == Feedforward::PolicyTorque && parameters_.ocs2.constraints.inverseDynamics.activate) {
    // The MPC's own torque. It is a physical torque only inside the torque window,
    // where the joint rows of the inverse dynamics hold; past it (a solve slower than
    // the window) tau is an unconstrained input, so take the plan's inverse dynamics
    // instead, which is what those rows would have made it.
    const auto& window = mpc_interface_->getTorqueWindow();
    const auto& policy_times = mrt_interface_->getPolicy().timeTrajectory_;
    const double policy_start = policy_times.front();
    if (window.containsInterpolatedInput(policy_time, policy_times)) {
      command.feedforward = control_model_->getJointTorques(policy_input);
    } else {
      ++torque_window_overruns_;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "%s policy evaluated %.3f s into its horizon; its interpolation exceeds the %.3f s torque window (%lu ticks so far); using the "
        "plan's inverse dynamics. Raise ocs2.constraints.inverseDynamics.torqueConstraintNodes or speed up the MPC.",
        kTag, policy_time - policy_start, window.duration(), static_cast<unsigned long>(torque_window_overruns_.load()));
      command.feedforward = plan_inverse_dynamics(
        control_model_->getGeneralizedCoordinates(policy_state), control_model_->getGeneralizedVelocities(policy_state));
    }
  } else {
    // Also avoid executing free policy-torque variables when the RNEA equalities are disabled.
    // Evaluate at the measured configuration and velocity.
    command.feedforward = plan_inverse_dynamics(
      control_model_->getGeneralizedCoordinates(observation.state), control_model_->getGeneralizedVelocities(observation.state));
  }
  clamp_torque(command.feedforward);
  return command;
}

void HumanoidWholeBodyInverseDynamicsMpcController::configure_tracking()
{
  tracking_publisher_.reset();
  tracking_publisher_raw_.reset();
  tracking_names_publisher_.reset();
  solver_publisher_.reset();
  solver_names_publisher_.reset();
  solver_diag_problem_.reset();
  if (parameters_.diagnostics.trackingRate <= 0.0) {
    return;
  }
  const double control_rate = get_update_rate() > 0 ? static_cast<double>(get_update_rate()) : 1000.0;
  tracking_decimation_ = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(control_rate / parameters_.diagnostics.trackingRate)));
  tracking_tick_ = 0;

  const auto& model = control_pinocchio_->getModel();
  tracking_foot_frames_.clear();
  for (const auto& frame : parameters_.ocs2.model.contactNames6DoF) {
    if (!model.existFrame(frame)) {
      throw std::invalid_argument("tracking debug: contact frame " + frame + " is not in the model");
    }
    tracking_foot_frames_.push_back(model.getFrameId(frame));
  }

  tracking_names_ = {"t", "phase", "policy_valid", "plan_mode", "policy_t0"};
  for (const char* group : {"cmd_q", "cmd_qd", "cmd_tau", "plan_q", "plan_v", "plan_tau", "meas_q", "meas_v", "meas_tau"}) {
    for (const auto& joint : parameters_.robot.jointNames) {
      tracking_names_.push_back(std::string(group) + "/" + joint);
    }
  }
  // Base: [x y z yaw pitch roll] and [vx vy vz yaw_rate pitch_rate roll_rate] (world, Euler rates), as in the state.
  for (const char* group : {"plan_base", "obs_base"}) {
    for (const char* axis : {"x", "y", "z", "yaw", "pitch", "roll", "vx", "vy", "vz", "dyaw", "dpitch", "droll"}) {
      tracking_names_.push_back(std::string(group) + "/" + axis);
    }
  }
  // Feet: contact-frame position and linear velocity in world axes.
  for (const char* group : {"plan_foot", "obs_foot"}) {
    for (const auto& frame : parameters_.ocs2.model.contactNames6DoF) {
      for (const char* axis : {"x", "y", "z", "vx", "vy", "vz"}) {
        tracking_names_.push_back(std::string(group) + "/" + frame + "/" + axis);
      }
    }
  }
  for (const auto& frame : parameters_.ocs2.model.contactNames6DoF) {
    for (const char* axis : {"fx", "fy", "fz", "mx", "my", "mz"}) {
      tracking_names_.push_back("plan_wrench/" + frame + "/" + axis);
    }
  }
  // Rolling wheels: gap of the tire's lowest point and the tread slip velocity at it, from the
  // observed state (WheelContactKinematics.h), plus the simulator's measured contact.
  if (parameters_.ocs2.constraints.wheelContact.activate) {
    for (const auto& frame : parameters_.ocs2.model.contactNames6DoF) {
      for (const char* name : {"gap", "slip_lon", "slip_lat"}) {
        tracking_names_.push_back("obs_wheel/" + frame + "/" + name);
      }
    }
  }
  for (const auto& sensor : parameters_.diagnostics.contactSensorNames) {
    tracking_names_.push_back("meas_contact/" + sensor + "/normal_force");
    tracking_names_.push_back("meas_contact/" + sensor + "/contact");
  }
  tracking_row_.data.assign(tracking_names_.size(), std::numeric_limits<double>::quiet_NaN());

  tracking_publisher_raw_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/tracking", rclcpp::SensorDataQoS());
  tracking_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(tracking_publisher_raw_);
  tracking_names_publisher_ =
    get_node()->create_publisher<std_msgs::msg::String>("~/tracking_names", rclcpp::QoS(1).transient_local().reliable());
  std_msgs::msg::String names;
  for (std::size_t i = 0; i < tracking_names_.size(); ++i) {
    names.data += (i ? "," : "") + tracking_names_[i];
  }
  tracking_names_publisher_->publish(names);
  RCLCPP_INFO(get_node()->get_logger(), "%s tracking debug: %zu columns on ~/tracking every %zu ticks", kTag,
              tracking_names_.size(), tracking_decimation_);

  // Solver metrics, published by the solver thread after each solve.
  const auto& problem = mpc_interface_->getOptimalControlProblem();
  solver_diag_problem_ = std::make_unique<ocs2::OptimalControlProblem>(problem);
  const auto sorted_names = [](const auto& map) {
    std::vector<std::pair<std::string, std::size_t>> v(map.begin(), map.end());
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    return v;
  };
  solver_eq_terms_ = sorted_names(problem.equalityConstraintPtr->getTermNameMap());
  const auto names_of = [&](const auto& map) {
    std::vector<std::string> out;
    for (const auto& [name, index] : sorted_names(map)) {
      out.push_back(name);
    }
    return out;
  };
  solver_cost_terms_ = names_of(problem.costPtr->getTermNameMap());
  solver_state_cost_terms_ = names_of(problem.stateCostPtr->getTermNameMap());
  solver_soft_terms_ = names_of(problem.softConstraintPtr->getTermNameMap());
  solver_state_soft_terms_ = names_of(problem.stateSoftConstraintPtr->getTermNameMap());
  std::vector<std::string> solver_names = {"t", "advance_ms", "merit", "cost", "dyn_sse", "eq_sse", "ineq_sse",
                                           "eq_lagrangian", "ineq_lagrangian", "dual_sse"};
  for (const auto& [name, index] : solver_eq_terms_) {
    solver_names.push_back("eq_sse/" + name);
  }
  // The same over the first nodes only: those drive the command, the rest is the tail of the plan.
  solver_names.push_back("eq_sse_near");
  for (const auto& [name, index] : solver_eq_terms_) {
    solver_names.push_back("eq_sse_near/" + name);
  }
  for (const auto& name : solver_cost_terms_) solver_names.push_back("cost/" + name);
  for (const auto& name : solver_state_cost_terms_) solver_names.push_back("state_cost/" + name);
  for (const auto& name : solver_soft_terms_) solver_names.push_back("soft/" + name);
  for (const auto& name : solver_state_soft_terms_) solver_names.push_back("state_soft/" + name);
  for (const auto* name : {"grid_nodes", "grid_min_dt", "grid_max_dt", "grid_fine_max_dt"}) solver_names.push_back(name);
  solver_row_.data.assign(solver_names.size(), std::numeric_limits<double>::quiet_NaN());
  solver_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/solver", rclcpp::QoS(200).best_effort());
  solver_names_publisher_ =
    get_node()->create_publisher<std_msgs::msg::String>("~/solver_names", rclcpp::QoS(1).transient_local().reliable());
  std_msgs::msg::String solver_header;
  for (std::size_t i = 0; i < solver_names.size(); ++i) {
    solver_header.data += (i ? "," : "") + solver_names[i];
  }
  solver_names_publisher_->publish(solver_header);
}

void HumanoidWholeBodyInverseDynamicsMpcController::publish_solver_metrics(double advance_ms)
{
  auto* solver = mpc_solver_->getSolverPtr();
  const auto& performance = solver->getPerformanceIndeces();
  const auto& metrics = solver->getSolutionMetrics();
  auto& row = solver_row_.data;
  std::fill(row.begin(), row.end(), std::numeric_limits<double>::quiet_NaN());
  std::size_t k = 0;
  const auto put = [&](double value) { row[k++] = value; };
  solver->getPrimalSolution(solver->getFinalTime(), &solver_primal_);
  put(solver_primal_.timeTrajectory_.empty() ? std::numeric_limits<double>::quiet_NaN() : solver_primal_.timeTrajectory_.front());
  put(advance_ms);
  put(performance.merit);
  put(performance.cost);
  put(performance.dynamicsViolationSSE);
  put(performance.equalityConstraintsSSE);
  put(performance.inequalityConstraintsSSE);
  put(performance.equalityLagrangian);
  put(performance.inequalityLagrangian);
  put(performance.dualFeasibilitiesSSE);
  for (const auto& [name, index] : solver_eq_terms_) {
    double sse = 0.0;
    for (const auto& node : metrics.intermediates) {
      if (index < node.stateInputEqConstraint.size()) {
        sse += node.stateInputEqConstraint[index].squaredNorm();
      }
    }
    put(sse);
  }
  constexpr std::size_t kNearNodes = 3;
  const std::size_t near_nodes = std::min(kNearNodes, metrics.intermediates.size());
  const std::size_t near_total = k;
  put(0.0);
  for (const auto& [name, index] : solver_eq_terms_) {
    double sse = 0.0;
    for (std::size_t n = 0; n < near_nodes; ++n) {
      const auto& node = metrics.intermediates[n];
      if (index < node.stateInputEqConstraint.size()) {
        sse += node.stateInputEqConstraint[index].squaredNorm();
      }
    }
    row[near_total] += sse;
    put(sse);
  }
  // Cost terms at the first node of the new solution, on the private problem copy. A diagnostic
  // must never take the solver thread down, so evaluation errors leave the columns NaN.
  try {
    if (!solver_primal_.timeTrajectory_.empty() && !solver_primal_.inputTrajectory_.empty()) {
      const double t = solver_primal_.timeTrajectory_.front();
      const auto& x = solver_primal_.stateTrajectory_.front();
      const auto& u = solver_primal_.inputTrajectory_.front();
      const auto& targets = solver->getReferenceManager().getTargetTrajectories();
      auto& problem = *solver_diag_problem_;
      problem.preComputationPtr->request(ocs2::Request::Cost + ocs2::Request::SoftConstraint, t, x, u);
      for (const auto& name : solver_cost_terms_) {
        put(problem.costPtr->get<ocs2::StateInputCost>(name).getValue(t, x, u, targets, *problem.preComputationPtr));
      }
      for (const auto& name : solver_state_cost_terms_) {
        put(problem.stateCostPtr->get<ocs2::StateCost>(name).getValue(t, x, targets, *problem.preComputationPtr));
      }
      for (const auto& name : solver_soft_terms_) {
        put(problem.softConstraintPtr->get<ocs2::StateInputCost>(name).getValue(t, x, u, targets, *problem.preComputationPtr));
      }
      for (const auto& name : solver_state_soft_terms_) {
        put(problem.stateSoftConstraintPtr->get<ocs2::StateCost>(name).getValue(t, x, targets, *problem.preComputationPtr));
      }
    }
  } catch (const std::exception&) {
  }
  // Append mesh diagnostics independently of optional cost-evaluation failures.
  k = row.size() - 4;
  put(static_cast<double>(solver_primal_.timeTrajectory_.size()));
  double min_dt = std::numeric_limits<double>::infinity(), max_dt = 0.0, fine_max_dt = 0.0;
  const auto& times = solver_primal_.timeTrajectory_;
  const double fine = parameters_.ocs2.solver.sqp.timeGrid.fineHorizon;
  for (std::size_t i = 1; i < times.size(); ++i) {
    const double dt = times[i] - times[i - 1];
    if (dt <= 1e-9) continue;
    min_dt = std::min(min_dt, dt);
    max_dt = std::max(max_dt, dt);
    if (times[i - 1] < times.front() + fine - 1e-9) fine_max_dt = std::max(fine_max_dt, dt);
  }
  put(std::isfinite(min_dt) ? min_dt : 0.0);
  put(max_dt);
  put(fine_max_dt);
  solver_publisher_->publish(solver_row_);
}

void HumanoidWholeBodyInverseDynamicsMpcController::publish_tracking(
  const ocs2::SystemObservation& observation, const JointActionCommand& command)
{
  auto& row = tracking_row_.data;
  std::fill(row.begin(), row.end(), std::numeric_limits<double>::quiet_NaN());
  const auto nj = static_cast<Eigen::Index>(parameters_.robot.jointNames.size());
  std::size_t k = 0;
  const auto put = [&](double value) { row[k++] = value; };
  const auto put_vec = [&](const vector_t& v, Eigen::Index n) {
    for (Eigen::Index i = 0; i < n; ++i) {
      put(i < v.size() ? v[i] : std::numeric_limits<double>::quiet_NaN());
    }
  };
  const auto skip = [&](std::size_t n) { k += n; };
  const bool plan = tracking_policy_valid_ && tracking_policy_state_.size() > 0;

  put(observation.time);
  put(static_cast<double>(static_cast<int>(bring_up_phase_.load())));
  put(plan ? 1.0 : 0.0);
  put(plan ? static_cast<double>(tracking_policy_mode_) : std::numeric_limits<double>::quiet_NaN());
  put(active_policy_ready_ ? mrt_interface_->getPolicy().timeTrajectory_.front() : std::numeric_limits<double>::quiet_NaN());

  put_vec(command.policy_position, nj);
  put_vec(command.policy_velocity, nj);
  put_vec(command.feedforward, nj);
  if (plan) {
    put_vec(control_model_->getJointAngles(tracking_policy_state_), nj);
    put_vec(control_model_->getJointVelocities(tracking_policy_state_, tracking_policy_input_), nj);
    put_vec(control_model_->getJointTorques(tracking_policy_input_), nj);
  } else {
    skip(3 * static_cast<std::size_t>(nj));
  }
  put_vec(control_model_->getJointAngles(observation.state), nj);
  put_vec(control_model_->getJointVelocities(observation.state, observation.input), nj);
  const auto effort_offset = interface_offset(parameters_.robot.stateInterfaces, hardware_interface::HW_IF_EFFORT);
  const std::size_t stride = parameters_.robot.stateInterfaces.size();
  for (Eigen::Index i = 0; i < nj; ++i) {
    const std::size_t index = static_cast<std::size_t>(i) * stride + (effort_offset ? *effort_offset : 0);
    const auto value = effort_offset && index < state_interfaces_.size() ? state_interfaces_[index].get_optional() : std::nullopt;
    put(value ? *value : std::numeric_limits<double>::quiet_NaN());
  }

  const auto base = [&](const vector_t& state) {
    put_vec(control_model_->getBasePose(state), 6);
    put_vec(control_model_->getBaseComVelocity(state), 6);
  };
  if (plan) {
    base(tracking_policy_state_);
  } else {
    skip(12);
  }
  base(observation.state);

  auto& data = control_pinocchio_->getData();
  const auto& model = control_pinocchio_->getModel();
  const auto feet = [&](const vector_t& state) {
    pinocchio::forwardKinematics(model, data, control_model_->getGeneralizedCoordinates(state),
                                 control_model_->getGeneralizedVelocities(state));
    for (const auto frame : tracking_foot_frames_) {
      const auto& placement = pinocchio::updateFramePlacement(model, data, frame);
      const auto velocity = pinocchio::getFrameVelocity(model, data, frame, pinocchio::LOCAL_WORLD_ALIGNED);
      put_vec(placement.translation(), 3);
      put_vec(velocity.linear(), 3);
    }
  };
  const std::size_t foot_columns = 6 * tracking_foot_frames_.size();
  if (plan) {
    feet(tracking_policy_state_);
  } else {
    skip(foot_columns);
  }
  feet(observation.state);
  if (plan) {
    for (std::size_t c = 0; c < tracking_foot_frames_.size(); ++c) {
      put_vec(control_model_->getContactWrench(tracking_policy_input_, c), 6);
    }
  } else {
    skip(6 * tracking_foot_frames_.size());
  }
  if (parameters_.ocs2.constraints.wheelContact.activate) {
    const auto& wheel = parameters_.ocs2.constraints.wheelContact;
    const Eigen::Vector3d axle_local = Eigen::Vector3d(wheel.axleAxis[0], wheel.axleAxis[1], wheel.axleAxis[2]).normalized();
    const Eigen::Vector3d normal(0.0, 0.0, 1.0);
    pinocchio::forwardKinematics(model, data, control_model_->getGeneralizedCoordinates(observation.state),
                                 control_model_->getGeneralizedVelocities(observation.state));
    for (const auto frame : tracking_foot_frames_) {
      const auto& placement = pinocchio::updateFramePlacement(model, data, frame);
      const auto velocity = pinocchio::getFrameVelocity(model, data, frame, pinocchio::LOCAL_WORLD_ALIGNED);
      const Eigen::Vector3d axle = placement.rotation() * axle_local;
      const Eigen::Vector3d rho = ocs2::humanoid::wheel::contactOffset<double>(axle, normal, wheel.radius);
      const Eigen::Vector3d slip = velocity.linear() + velocity.angular().cross(rho);
      put(normal.dot(placement.translation() + rho));
      put(ocs2::humanoid::wheel::longitudinalDirection<double>(axle, normal).dot(slip));
      put(ocs2::humanoid::wheel::lateralDirection<double>(axle, normal).dot(slip));
    }
  }
  for (const auto& sensor : parameters_.diagnostics.contactSensorNames) {
    const auto force = get_state_interface_value(sensor, "normal_force");
    const auto contact = get_state_interface_value(sensor, "contact");
    put(force ? *force : std::numeric_limits<double>::quiet_NaN());
    put(contact ? *contact : std::numeric_limits<double>::quiet_NaN());
  }
  (void)tracking_publisher_->try_publish(tracking_row_);
}

void HumanoidWholeBodyInverseDynamicsMpcController::write_joint_action_command(
  const vector_t& q_des, const vector_t& qd_des, const vector_t& tau_ff)
{
  // effort_pd: the hardware evaluates kp*(q_des-q)+kd*(qd_des-qd)+tau_ff every physics
  // step. Three command interfaces per joint.
  const size_t mpc_joint_count = parameters_.robot.jointNames.size();
  const size_t fixed_joint_count = parameters_.ocs2.model.fixedJointNames.size();
  if (command_interfaces_.size() < (mpc_joint_count + fixed_joint_count) * 3) {
    return;
  }
  auto safe = [](double v) { return std::isfinite(v) ? v : 0.0; };
  for (size_t i = 0; i < mpc_joint_count; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    (void)command_interfaces_[3 * i + 0].set_value(safe(i < static_cast<size_t>(q_des.size()) ? q_des[idx] : 0.0));
    (void)command_interfaces_[3 * i + 1].set_value(safe(i < static_cast<size_t>(qd_des.size()) ? qd_des[idx] : 0.0));
    (void)command_interfaces_[3 * i + 2].set_value(safe(i < static_cast<size_t>(tau_ff.size()) ? tau_ff[idx] : 0.0));
  }
  // Servo gains, every tick: the hardware runs the PD with whatever was written last.
  const vector_t& kp = servo_kp_override_.size() == joint_kp_.size() ? servo_kp_override_ : joint_kp_;
  const vector_t& kd = servo_kd_override_.size() == joint_kd_.size() ? servo_kd_override_ : joint_kd_;
  for (size_t i = 0; 2 * i + 1 < gain_command_index_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    (void)command_interfaces_[gain_command_index_[2 * i]].set_value(kp[idx]);
    (void)command_interfaces_[gain_command_index_[2 * i + 1]].set_value(kd[idx]);
  }
  // Joints excluded from the MPC model (wrists): hold zero, soft hardware gains.
  for (size_t i = 0; i < fixed_joint_count; ++i) {
    const size_t base = 3 * (mpc_joint_count + i);
    (void)command_interfaces_[base + 0].set_value(0.0);
    (void)command_interfaces_[base + 1].set_value(0.0);
    (void)command_interfaces_[base + 2].set_value(0.0);
  }
}

}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidWholeBodyInverseDynamicsMpcController,
  controller_interface::ChainableControllerInterface)
