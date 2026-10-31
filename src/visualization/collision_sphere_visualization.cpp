#include <pinocchio/fwd.hpp>

#include "legged_robot_mpc_controller/visualization/collision_sphere_visualization.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace legged_robot_mpc_controller::visualization
{

CollisionSphereVisualization::CollisionSphereVisualization(
  const ocs2::PinocchioInterface& pinocchioInterface,
  const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
  rclcpp_lifecycle::LifecycleNode& node, Settings settings)
: pinocchio_interface_(pinocchioInterface),
  mpc_robot_model_(mpcRobotModel.clone()), node_(node), settings_(std::move(settings))
{
  if (settings_.frame_id.empty() || settings_.marker_topic.empty() ||
      !std::isfinite(settings_.alpha) || settings_.alpha <= 0.0 || settings_.alpha > 1.0) {
    throw std::invalid_argument("Invalid collision sphere visualization frame, topic or alpha.");
  }
  const auto& model = pinocchio_interface_.getModel();
  std::unordered_set<std::string> names;
  for (const auto& sphere : settings_.spheres) {
    if (sphere.name.empty() || !names.insert(sphere.group + "/" + sphere.name).second ||
        !sphere.center.allFinite() || !std::isfinite(sphere.radius) || sphere.radius <= 0.0 ||
        !model.existFrame(sphere.frame)) {
      throw std::invalid_argument("Invalid collision visualization sphere: " + sphere.name);
    }
    frame_ids_.push_back(model.getFrameId(sphere.frame));
  }
  publisher_ = node_.create_publisher<Message>(
    settings_.marker_topic, rclcpp::QoS(1).best_effort().durability_volatile());
}

bool CollisionSphereVisualization::hasSubscribers() const
{
  return publisher_->get_subscription_count() + publisher_->get_intra_process_subscription_count() > 0;
}

void CollisionSphereVisualization::publish(const ocs2::vector_t& observedState)
{
  if (hasSubscribers()) publisher_->publish(createMessage(observedState));
}

CollisionSphereVisualization::Message CollisionSphereVisualization::createMessage(
  const ocs2::vector_t& observedState)
{
  if (observedState.size() != static_cast<Eigen::Index>(mpc_robot_model_->getStateDim()) ||
      !observedState.allFinite()) {
    throw std::invalid_argument("Invalid observed state for collision sphere visualization.");
  }
  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  const auto q = mpc_robot_model_->getGeneralizedCoordinates(observedState);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  Message message;
  const auto stamp = node_.get_clock()->now();
  for (std::size_t i = 0; i < settings_.spheres.size(); ++i) {
    const auto& sphere = settings_.spheres[i];
    const Eigen::Vector3d center = data.oMf[frame_ids_[i]].act(sphere.center);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = settings_.frame_id;
    marker.header.stamp = stamp;
    marker.ns = "collision_" + sphere.group;
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * sphere.radius;
    marker.color.r = sphere.group == "arms" ? 1.0F : 0.15F;
    marker.color.g = sphere.group == "legs" ? 0.85F : 0.55F;
    marker.color.b = sphere.group == "body" ? 1.0F : 0.15F;
    marker.color.a = static_cast<float>(settings_.alpha);
    marker.text = sphere.name;
    // Expire stale poses after a stopped controller, even if no further messages arrive.
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    message.markers.push_back(std::move(marker));
  }
  return message;
}

}  // namespace legged_robot_mpc_controller::visualization
