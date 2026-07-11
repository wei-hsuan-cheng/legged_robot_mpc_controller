#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h>

#include "legged_robot_mpc_controller/common/heading_reference.hpp"
#include "legged_robot_mpc_controller/common/ros2_procedural_mpc_motion_manager.hpp"
#include "legged_robot_mpc_controller/humanoid_centroidal_mpc_controller_parameters.hpp"
#include "legged_robot_mpc_controller/visualization/performance_visualization.hpp"

namespace legged_robot_mpc_controller
{

class HumanoidCentroidalMpcController : public controller_interface::ChainableControllerInterface
{
public:
  using Params = humanoid_centroidal_mpc_controller::Params;
  using ParamListener = humanoid_centroidal_mpc_controller::ParamListener;

  ~HumanoidCentroidalMpcController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

private:
  using vector_t = ocs2::vector_t;

  struct JointActionCommand
  {
    vector_t policy_position;
    vector_t policy_velocity;
    vector_t feedforward;
  };

  controller_interface::InterfaceConfiguration make_joint_interface_configuration(
    const std::vector<std::string>& interface_names) const;
  std::vector<std::string> floating_base_state_interface_names() const;
  bool read_joint_state(vector_t& q, vector_t& v);
  std::optional<double> get_state_interface_value(
    const std::string& prefix_name,
    const std::string& interface_name) const;
  ocs2::SystemObservation build_observation(const rclcpp::Time& time);
  ocs2::TargetTrajectories current_observation_to_reset_trajectory(
    const ocs2::SystemObservation& observation);
  void start_solver_thread(const ocs2::SystemObservation& initial_observation);
  void stop_solver_thread();
  void solver_worker();
  vector_t compute_weight_compensating_torque(const ocs2::SystemObservation& observation);
  JointActionCommand compute_mpc_joint_action(const ocs2::SystemObservation& observation);
  void write_joint_action_command(
    const vector_t& q_des, const vector_t& qd_des, const vector_t& tau_ff);

  std::shared_ptr<ParamListener> param_listener_;
  Params parameters_;

  std::unique_ptr<ocs2::humanoid::CentroidalMpcInterface> mpc_interface_;
  std::unique_ptr<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>> control_model_;
  // Controller-owned pinocchio copy for the observation momentum mapping and the
  // feedforward inverse dynamics (the solver owns its own instances).
  std::unique_ptr<ocs2::PinocchioInterface> control_pinocchio_;
  std::unique_ptr<ocs2::humanoid::CentroidalMpcTargetTrajectoriesCalculator> target_trajectories_calculator_;
  std::shared_ptr<Ros2ProceduralMpcMotionManager> motion_manager_;
  std::unique_ptr<ocs2::MPC_BASE> mpc_solver_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_interface_;
  std::unique_ptr<visualization::PerformanceVisualization> performance_visualization_;

  // Heading hold for velocity-commanded walking; only touched from the solver thread.
  common::HeadingReference heading_reference_;

  // Observation velocity low-pass state (see build_observation).
  vector_t filtered_generalized_velocity_;
  double last_visualization_time_{-1.0};

  std::jthread solver_thread_;
  std::atomic_bool terminate_solver_thread_{false};

  vector_t initial_observation_state_;
  vector_t mpc_joint_kp_;
  vector_t mpc_joint_kd_;
  vector_t fixed_joint_kp_;
  vector_t fixed_joint_kd_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_
