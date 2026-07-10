#include "legged_robot_mpc_controller/visualization/optimized_state_trajectory_visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace legged_robot_mpc_controller::visualization
{
namespace
{

constexpr std::array<float, 3> kStartColor{0.0F, 0.447F, 0.741F};
constexpr std::array<float, 3> kEndColor{0.85F, 0.325F, 0.098F};
constexpr double kGoldenRatioConjugate = 0.6180339887498949;

struct TrajectoryColorGradient
{
  std::array<float, 3> start;
  std::array<float, 3> end;
};

std::array<float, 3> rgbToHsv(const std::array<float, 3>& rgb)
{
  const float max_value = std::max({rgb[0], rgb[1], rgb[2]});
  const float min_value = std::min({rgb[0], rgb[1], rgb[2]});
  const float delta = max_value - min_value;

  float hue = 0.0F;
  if (delta > 0.0F) {
    if (max_value == rgb[0]) {
      hue = std::fmod((rgb[1] - rgb[2]) / delta, 6.0F);
    } else if (max_value == rgb[1]) {
      hue = ((rgb[2] - rgb[0]) / delta) + 2.0F;
    } else {
      hue = ((rgb[0] - rgb[1]) / delta) + 4.0F;
    }

    hue /= 6.0F;
    if (hue < 0.0F) {
      hue += 1.0F;
    }
  }

  const float saturation = max_value > 0.0F ? delta / max_value : 0.0F;
  return {hue, saturation, max_value};
}

std::array<float, 3> hsvToRgb(const std::array<float, 3>& hsv)
{
  const float hue = hsv[0] - std::floor(hsv[0]);
  const float saturation = std::clamp(hsv[1], 0.0F, 1.0F);
  const float value = std::clamp(hsv[2], 0.0F, 1.0F);

  if (saturation <= 0.0F) {
    return {value, value, value};
  }

  const float scaled_hue = hue * 6.0F;
  const int sector = static_cast<int>(std::floor(scaled_hue));
  const float fraction = scaled_hue - static_cast<float>(sector);

  const float p = value * (1.0F - saturation);
  const float q = value * (1.0F - saturation * fraction);
  const float t = value * (1.0F - saturation * (1.0F - fraction));

  switch (sector % 6) {
    case 0:
      return {value, t, p};
    case 1:
      return {q, value, p};
    case 2:
      return {p, value, t};
    case 3:
      return {p, q, value};
    case 4:
      return {t, p, value};
    default:
      return {value, p, q};
  }
}

std::array<float, 3> rotateHue(const std::array<float, 3>& rgb, double hue_offset)
{
  auto hsv = rgbToHsv(rgb);
  const float shifted_hue = hsv[0] + static_cast<float>(hue_offset);
  hsv[0] = shifted_hue - std::floor(shifted_hue);
  return hsvToRgb(hsv);
}

TrajectoryColorGradient trajectoryColorGradient(std::size_t trajectory_index)
{
  // Golden-angle ordering keeps adjacent trajectory colors far apart.
  const double hue_offset =
    std::fmod(static_cast<double>(trajectory_index) * kGoldenRatioConjugate, 1.0);

  return {rotateHue(kStartColor, hue_offset), rotateHue(kEndColor, hue_offset)};
}

std_msgs::msg::ColorRGBA trajectoryColor(double progress, const TrajectoryColorGradient& gradient)
{
  const float t = static_cast<float>(std::clamp(progress, 0.0, 1.0));
  std_msgs::msg::ColorRGBA color;
  color.r = (1.0F - t) * gradient.start[0] + t * gradient.end[0];
  color.g = (1.0F - t) * gradient.start[1] + t * gradient.end[1];
  color.b = (1.0F - t) * gradient.start[2] + t * gradient.end[2];
  color.a = 1.0F;
  return color;
}

builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time& time)
{
  const std::int64_t nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  msg.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return msg;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& position)
{
  geometry_msgs::msg::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

}  // namespace

OptimizedStateTrajectoryVisualization::OptimizedStateTrajectoryVisualization(
  ocs2::PinocchioInterface pinocchioInterface,
  const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
  rclcpp_lifecycle::LifecycleNode& node,
  Settings settings)
: pinocchio_interface_(std::move(pinocchioInterface)),
  mpc_robot_model_(mpcRobotModel.clone()),
  node_(node),
  settings_(std::move(settings))
{
  const auto& model = pinocchio_interface_.getModel();
  frame_names_ = settings_.frame_names;
  if (frame_names_.empty()) {
    // Default to the contact frames so the display is never empty.
    frame_names_ = mpc_robot_model_->modelSettings.contactNames6DoF;
  }
  // Skip frames absent from the (reduced) MPC model, matching the legacy visualizer.
  std::vector<std::string> available_frame_names;
  available_frame_names.reserve(frame_names_.size());
  frame_ids_.reserve(frame_names_.size());
  for (const auto& frame_name : frame_names_) {
    if (!model.existFrame(frame_name)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "[OptimizedStateTrajectoryVisualization] frame '%s' does not exist in the MPC Pinocchio "
        "model; skipping it.",
        frame_name.c_str());
      continue;
    }
    frame_ids_.push_back(model.getFrameId(frame_name));
    available_frame_names.push_back(frame_name);
  }
  frame_names_ = std::move(available_frame_names);
  if (frame_names_.empty()) {
    throw std::invalid_argument(
      "Optimized state trajectory visualization has no valid frames to track.");
  }
  if (settings_.frame_id.empty()) {
    throw std::invalid_argument("Optimized state trajectory frame ID must not be empty.");
  }
  if (!std::isfinite(settings_.line_width) || settings_.line_width <= 0.0 ||
      !std::isfinite(settings_.point_scale) || settings_.point_scale <= 0.0) {
    throw std::invalid_argument("Optimized state trajectory marker sizes must be finite and positive.");
  }

  marker_publisher_ = node_.create_publisher<Message>(
    settings_.marker_topic,
    rclcpp::QoS(1).reliable().transient_local());
}

void OptimizedStateTrajectoryVisualization::publish(
  const ocs2::vector_array_t& stateTrajectory)
{
  const auto generalized_coordinates = extractGeneralizedCoordinateTrajectory(stateTrajectory);
  if (generalized_coordinates.empty()) {
    return;
  }

  marker_publisher_->publish(createMessage(generalized_coordinates));
}

ocs2::vector_array_t OptimizedStateTrajectoryVisualization::extractGeneralizedCoordinateTrajectory(
  const ocs2::vector_array_t& stateTrajectory) const
{
  const auto gen_coordinates_dim =
    static_cast<Eigen::Index>(mpc_robot_model_->getGenCoordinatesDim());
  if (gen_coordinates_dim <= 0 || stateTrajectory.empty()) {
    return {};
  }

  ocs2::vector_array_t generalized_coordinates;
  generalized_coordinates.reserve(stateTrajectory.size());
  for (const auto& state : stateTrajectory) {
    if (state.size() < gen_coordinates_dim) {
      throw std::runtime_error("Optimized state sample is shorter than the generalized coordinates.");
    }

    auto q = mpc_robot_model_->getGeneralizedCoordinates(state);
    if (!q.allFinite()) {
      throw std::runtime_error("Optimized state trajectory contains a non-finite joint position.");
    }
    generalized_coordinates.push_back(std::move(q));
  }

  return generalized_coordinates;
}

OptimizedStateTrajectoryVisualization::Message OptimizedStateTrajectoryVisualization::createMessage(
  const ocs2::vector_array_t& generalizedCoordinateTrajectory)
{
  Message message;
  const auto stamp = node_.get_clock()->now();

  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = settings_.frame_id;
  delete_all.header.stamp = toTimeMsg(stamp);
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  message.markers.push_back(std::move(delete_all));

  std::vector<visualization_msgs::msg::Marker> line_markers;
  std::vector<visualization_msgs::msg::Marker> point_markers;
  std::vector<TrajectoryColorGradient> color_gradients;
  line_markers.reserve(frame_ids_.size());
  point_markers.reserve(frame_ids_.size());
  color_gradients.reserve(frame_ids_.size());

  for (std::size_t frame_index = 0; frame_index < frame_ids_.size(); ++frame_index) {
    color_gradients.push_back(trajectoryColorGradient(frame_index));

    visualization_msgs::msg::Marker line;
    line.header.frame_id = settings_.frame_id;
    line.header.stamp = toTimeMsg(stamp);
    line.ns = "optimizedStateTrajectoryLine_" + frame_names_[frame_index];
    line.id = static_cast<int>(2 * frame_index);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = settings_.line_width;
    line.points.reserve(generalizedCoordinateTrajectory.size());
    line.colors.reserve(generalizedCoordinateTrajectory.size());

    visualization_msgs::msg::Marker points;
    points.header = line.header;
    points.ns = "optimizedStateTrajectorySamples_" + frame_names_[frame_index];
    points.id = static_cast<int>(2 * frame_index + 1);
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = settings_.point_scale;
    points.scale.y = settings_.point_scale;
    points.scale.z = settings_.point_scale;
    points.points.reserve(generalizedCoordinateTrajectory.size());
    points.colors.reserve(generalizedCoordinateTrajectory.size());

    line_markers.push_back(std::move(line));
    point_markers.push_back(std::move(points));
  }

  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  for (std::size_t index = 0; index < generalizedCoordinateTrajectory.size(); ++index) {
    pinocchio::forwardKinematics(model, data, generalizedCoordinateTrajectory[index]);
    pinocchio::updateFramePlacements(model, data);

    const double progress =
      generalizedCoordinateTrajectory.size() > 1 ?
      static_cast<double>(index) / static_cast<double>(generalizedCoordinateTrajectory.size() - 1) : 0.0;

    for (std::size_t frame_index = 0; frame_index < frame_ids_.size(); ++frame_index) {
      const auto point = toPoint(data.oMf[frame_ids_[frame_index]].translation());
      const auto color = trajectoryColor(progress, color_gradients[frame_index]);

      line_markers[frame_index].points.push_back(point);
      line_markers[frame_index].colors.push_back(color);
      point_markers[frame_index].points.push_back(point);
      point_markers[frame_index].colors.push_back(color);
    }
  }

  for (auto& line : line_markers) {
    message.markers.push_back(std::move(line));
  }
  for (auto& points : point_markers) {
    message.markers.push_back(std::move(points));
  }
  return message;
}

}  // namespace legged_robot_mpc_controller::visualization
