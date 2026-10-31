#ifndef LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__COLLISION_SPHERE_VISUALIZATION_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__COLLISION_SPHERE_VISUALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace legged_robot_mpc_controller::visualization
{

class CollisionSphereVisualization
{
public:
  struct Sphere
  {
    std::string name;
    std::string frame;
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double radius{0.0};
    std::string group;
  };

  struct Settings
  {
    bool activate{false};
    std::string marker_topic{"/performance_visualization/collisionSpheres"};
    std::string frame_id{"odom"};
    double alpha{0.25};
    std::vector<Sphere> spheres;
  };

  using Message = visualization_msgs::msg::MarkerArray;

  CollisionSphereVisualization(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
    rclcpp_lifecycle::LifecycleNode& node, Settings settings);

  bool hasSubscribers() const;
  void publish(const ocs2::vector_t& observedState);
  Message createMessage(const ocs2::vector_t& observedState);

private:
  ocs2::PinocchioInterface pinocchio_interface_;
  std::unique_ptr<ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>> mpc_robot_model_;
  rclcpp_lifecycle::LifecycleNode& node_;
  Settings settings_;
  std::vector<pinocchio::FrameIndex> frame_ids_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
};

}  // namespace legged_robot_mpc_controller::visualization

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__COLLISION_SPHERE_VISUALIZATION_HPP_
