#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"

#include <yaml-cpp/yaml.h>

namespace legged_robot_mpc_controller::common
{

ocs2::humanoid::GaitMap loadGaitMap(const std::string& gaitLibraryFile)
{
  const YAML::Node root = YAML::LoadFile(gaitLibraryFile);
  if (!root["list"]) {
    throw std::runtime_error(
      "[config_builder] gait library file has no 'list' key: " + gaitLibraryFile);
  }

  ocs2::humanoid::GaitMap gaitMap;
  for (const auto& gaitNameNode : root["list"]) {
    const auto gaitName = gaitNameNode.as<std::string>();
    const YAML::Node gait = root[gaitName];
    if (!gait || !gait["modeSequence"] || !gait["switchingTimes"]) {
      throw std::runtime_error(
        "[config_builder] gait '" + gaitName + "' is missing in " + gaitLibraryFile);
    }
    const auto modeSequence = gait["modeSequence"].as<std::vector<std::string>>();
    const auto switchingTimes = gait["switchingTimes"].as<std::vector<double>>();
    gaitMap.insert({gaitName, ocs2::humanoid::modeSequenceTemplateFromStrings(switchingTimes, modeSequence)});
  }
  return gaitMap;
}

ocs2::humanoid::StairClimbingConfig loadStairClimbingConfig(const std::string& stairClimbingFile)
{
  const YAML::Node root = YAML::LoadFile(stairClimbingFile);
  const YAML::Node node = root["stair_climbing"];
  if (!node) {
    throw std::runtime_error(
      "[config_builder] stair climbing file has no 'stair_climbing' key: " + stairClimbingFile);
  }

  ocs2::humanoid::StairClimbingConfig config;

  const YAML::Node stairs = node["stairs"];
  if (!stairs || !stairs["base_pos"] || !stairs["heights"] || !stairs["depths"]) {
    throw std::runtime_error(
      "[config_builder] stair_climbing.stairs needs base_pos, heights and depths: " + stairClimbingFile);
  }
  const auto basePos = stairs["base_pos"].as<std::vector<double>>();
  if (basePos.size() != 3) {
    throw std::runtime_error("[config_builder] stair_climbing.stairs.base_pos must have 3 entries");
  }
  config.stairsBasePosition << basePos[0], basePos[1], basePos[2];
  config.stairsYaw = stairs["yaw"].as<double>(0.0);
  config.startOffset = stairs["start_offset"].as<double>(0.0);
  config.stepHeights = stairs["heights"].as<std::vector<double>>();
  config.stepDepths = stairs["depths"].as<std::vector<double>>();
  config.stepWidth = stairs["width"].as<double>(config.stepWidth);

  if (const YAML::Node gait = node["gait"]) {
    config.initialStanceDuration = gait["initial_stance_duration"].as<double>(config.initialStanceDuration);
    config.swingDuration = gait["swing_duration"].as<double>(config.swingDuration);
    config.stanceDuration = gait["stance_duration"].as<double>(config.stanceDuration);
    config.finalStanceDuration = gait["final_stance_duration"].as<double>(config.finalStanceDuration);
    config.leftFootFirst = gait["left_foot_first"].as<bool>(config.leftFootFirst);
    config.bothFeetPerTread = gait["both_feet_per_tread"].as<bool>(config.bothFeetPerTread);
  }

  if (const YAML::Node footholds = node["footholds"]) {
    config.lateralOffset = footholds["lateral_offset"].as<double>(config.lateralOffset);
    config.treadInset = footholds["tread_inset"].as<double>(config.treadInset);
    config.approachStride = footholds["approach_stride"].as<double>(config.approachStride);
    config.approachStandoff = footholds["approach_standoff"].as<double>(config.approachStandoff);
    config.footholdTrackingWeight = footholds["tracking_weight"].as<double>(config.footholdTrackingWeight);
  }

  if (const YAML::Node base = node["base"]) {
    config.baseHeightAboveSupport = base["height_above_support"].as<double>(config.baseHeightAboveSupport);
  }

  return config;
}

ocs2::humanoid::TerrainFootholdPlannerSettings loadTerrainFootholdPlannerSettings(const std::string& stairClimbingFile)
{
  ocs2::humanoid::TerrainFootholdPlannerSettings settings;

  const YAML::Node root = YAML::LoadFile(stairClimbingFile);
  const YAML::Node node = root["terrain_walk"];
  if (!node) {
    return settings;  // defaults
  }

  settings.hipLateralOffset = node["hip_lateral_offset"].as<double>(settings.hipLateralOffset);
  settings.footMarginX = node["foot_margin_x"].as<double>(settings.footMarginX);
  settings.footMarginY = node["foot_margin_y"].as<double>(settings.footMarginY);
  settings.maxStepHeight = node["max_step_height"].as<double>(settings.maxStepHeight);
  settings.capturePointFeedbackGain = node["capture_point_feedback_gain"].as<double>(settings.capturePointFeedbackGain);
  settings.maxCapturePointOffset = node["max_capture_point_offset"].as<double>(settings.maxCapturePointOffset);
  settings.maxBaseHeightAboveSupport = node["max_base_height_above_support"].as<double>(settings.maxBaseHeightAboveSupport);
  settings.engageDistance = node["engage_distance"].as<double>(settings.engageDistance);
  settings.maxBaseLead = node["max_base_lead"].as<double>(settings.maxBaseLead);
  settings.footholdTrackingWeight = node["tracking_weight"].as<double>(settings.footholdTrackingWeight);
  settings.swingReferenceArrivalFraction = node["swing_reference_arrival_fraction"].as<double>(settings.swingReferenceArrivalFraction);

  return settings;
}

}  // namespace legged_robot_mpc_controller::common
