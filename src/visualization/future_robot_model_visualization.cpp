#include "legged_robot_mpc_controller/visualization/future_robot_model_visualization.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <urdf_model/model.h>

namespace legged_robot_mpc_controller::visualization
{
namespace
{

constexpr std::array<float, 3> kFutureGhostColor{0.765F, 0.665F, 0.905F};
constexpr std::array<float, 3> kTerminalStateColor{0.180F, 0.220F, 0.300F};

Eigen::Isometry3d visualOrigin(const urdf::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  double x, y, z, w;
  pose.rotation.getQuaternion(x, y, z, w);
  Eigen::Quaterniond rotation(w, x, y, z);
  rotation.normalize();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() << pose.position.x, pose.position.y, pose.position.z;
  return transform;
}

bool subscribed(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher)
{
  return publisher->get_subscription_count() + publisher->get_intra_process_subscription_count() > 0;
}

}  // namespace

FutureRobotModelVisualization::FutureRobotModelVisualization(
  ocs2::PinocchioInterface pinocchioInterface,
  const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
  rclcpp_lifecycle::LifecycleNode& node,
  Settings settings)
: pinocchio_interface_(std::move(pinocchioInterface)),
  mpc_robot_model_(mpcRobotModel.clone()),
  node_(node),
  settings_(std::move(settings))
{
  if (settings_.frame_id.empty() || !std::isfinite(settings_.interval) || settings_.interval <= 0.0) {
    throw std::invalid_argument("Future robot ghosts require a frame and a finite positive interval.");
  }
  const auto qos = rclcpp::QoS(1).best_effort();
  future_publisher_ = node_.create_publisher<Message>("/performance_visualization/optimizedStateGhosts", qos);
  terminal_state_publisher_ = node_.create_publisher<Message>("/performance_visualization/terminalStateGhost", qos);
  terminal_target_publisher_ = node_.create_publisher<Message>("/performance_visualization/terminalTargetGhost", qos);
  initializeVisuals();
  RCLCPP_INFO(
    node_.get_logger(), "Robot mesh ghosts: interval=%.3f s | visual_elements=%zu | subscriber-driven",
    settings_.interval, visuals_.size());
}

bool FutureRobotModelVisualization::hasSubscribers() const
{
  return !visuals_.empty() && (subscribed(future_publisher_) || subscribed(terminal_state_publisher_) ||
                             subscribed(terminal_target_publisher_));
}

void FutureRobotModelVisualization::initializeVisuals()
{
  const auto urdf = pinocchio_interface_.getUrdfModelPtr();
  if (!urdf) {
    RCLCPP_WARN(node_.get_logger(), "No URDF visuals available for future robot ghosts.");
    return;
  }
  const auto& model = pinocchio_interface_.getModel();
  for (const auto& [name, link] : urdf->links_) {
    std::optional<std::size_t> frame_id;
    for (std::size_t index = 0; index < model.frames.size(); ++index) {
      if (model.frames[index].name == name && (model.frames[index].type & pinocchio::FrameType::BODY) != 0) {
        frame_id = index;
        break;
      }
    }
    if (!frame_id || !link) continue;
    auto elements = link->visual_array;
    if (elements.empty() && link->visual) elements.push_back(link->visual);
    for (const auto& visual : elements) {
      if (!visual || !visual->geometry) continue;
      visualization_msgs::msg::Marker marker;
      marker.action = visualization_msgs::msg::Marker::ADD;
      switch (visual->geometry->type) {
        case urdf::Geometry::MESH: {
          const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
          if (!mesh || mesh->filename.empty()) continue;
          marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
          marker.mesh_resource = mesh->filename;
          marker.mesh_use_embedded_materials = false;
          marker.scale.x = std::abs(mesh->scale.x) > 1e-9 ? mesh->scale.x : 1.0;
          marker.scale.y = std::abs(mesh->scale.y) > 1e-9 ? mesh->scale.y : 1.0;
          marker.scale.z = std::abs(mesh->scale.z) > 1e-9 ? mesh->scale.z : 1.0;
          break;
        }
        case urdf::Geometry::BOX: {
          const auto box = std::dynamic_pointer_cast<urdf::Box>(visual->geometry);
          if (!box) continue;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.scale.x = box->dim.x;
          marker.scale.y = box->dim.y;
          marker.scale.z = box->dim.z;
          break;
        }
        case urdf::Geometry::CYLINDER: {
          const auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(visual->geometry);
          if (!cylinder) continue;
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          marker.scale.x = marker.scale.y = 2.0 * cylinder->radius;
          marker.scale.z = cylinder->length;
          break;
        }
        case urdf::Geometry::SPHERE: {
          const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(visual->geometry);
          if (!sphere) continue;
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * sphere->radius;
          break;
        }
        default:
          continue;
      }
      visuals_.push_back({*frame_id, visualOrigin(visual->origin), std::move(marker)});
    }
  }
}

FutureRobotModelVisualization::Message FutureRobotModelVisualization::emptyMessage(const rclcpp::Time& stamp) const
{
  Message message;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = settings_.frame_id;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  message.markers.push_back(std::move(clear));
  return message;
}

void FutureRobotModelVisualization::appendStateMarkers(
  Message& message, const rclcpp::Time& stamp, const ocs2::vector_t& state,
  const std::string& markerNamespace, const std::array<float, 3>& color, double alpha)
{
  if (state.size() != static_cast<Eigen::Index>(mpc_robot_model_->getStateDim()) || !state.allFinite()) {
    throw std::runtime_error("Robot ghost state has an invalid dimension or non-finite value.");
  }
  const auto q = mpc_robot_model_->getGeneralizedCoordinates(state);
  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  for (const auto& visual : visuals_) {
    auto marker = visual.marker;
    marker.header.frame_id = settings_.frame_id;
    marker.header.stamp = stamp;
    marker.ns = markerNamespace;
    marker.id = static_cast<int>(message.markers.size() - 1);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = alpha;
    Eigen::Isometry3d world_to_link = Eigen::Isometry3d::Identity();
    world_to_link.linear() = data.oMf[visual.frame_id].rotation();
    world_to_link.translation() = data.oMf[visual.frame_id].translation();
    const Eigen::Isometry3d pose = world_to_link * visual.link_to_visual;
    const Eigen::Quaterniond rotation(pose.linear());
    marker.pose.position.x = pose.translation().x();
    marker.pose.position.y = pose.translation().y();
    marker.pose.position.z = pose.translation().z();
    marker.pose.orientation.x = rotation.x();
    marker.pose.orientation.y = rotation.y();
    marker.pose.orientation.z = rotation.z();
    marker.pose.orientation.w = rotation.w();
    message.markers.push_back(std::move(marker));
  }
}

void FutureRobotModelVisualization::publish(
  const ocs2::scalar_array_t& times, const ocs2::vector_array_t& states,
  const ocs2::TargetTrajectories& targets)
{
  if (!hasSubscribers()) return;
  if (times.empty() || times.size() != states.size() || !std::is_sorted(times.begin(), times.end()) ||
      !std::all_of(times.begin(), times.end(), [](double time) { return std::isfinite(time); })) return;
  const auto stamp = node_.get_clock()->now();
  if (subscribed(future_publisher_)) {
    const double horizon = times.back() - times.front();
    const auto count = static_cast<std::size_t>(std::max(1.0, std::round(horizon / settings_.interval)));
    auto message = emptyMessage(stamp);
    message.markers.reserve(1 + count * visuals_.size());
    for (std::size_t index = 0; index < count; ++index) {
      // Match the legacy spacing: distribute samples over the actual MPC policy, including its end.
      const double time = times.front() + horizon * static_cast<double>(index + 1) / static_cast<double>(count);
      const auto state = ocs2::LinearInterpolation::interpolate(time, times, states);
      const double progress = count > 1 ? static_cast<double>(index) / static_cast<double>(count - 1) : 0.0;
      appendStateMarkers(message, stamp, state, "optimized_state_ghosts", kFutureGhostColor,
                         0.22 + (0.08 - 0.22) * progress);
    }
    future_publisher_->publish(message);
  }
  if (subscribed(terminal_state_publisher_)) {
    auto message = emptyMessage(stamp);
    appendStateMarkers(message, stamp, states.back(), "terminal_state_ghost", kTerminalStateColor, 0.40);
    terminal_state_publisher_->publish(message);
  }
  if (subscribed(terminal_target_publisher_)) {
    auto message = emptyMessage(stamp);
    if (!targets.empty()) {
      const auto state = targets.getDesiredState(times.back());
      appendStateMarkers(message, stamp, state, "terminal_target_ghost", kFutureGhostColor, 0.30);
    }
    terminal_target_publisher_->publish(message);
  }
}

}  // namespace legged_robot_mpc_controller::visualization
