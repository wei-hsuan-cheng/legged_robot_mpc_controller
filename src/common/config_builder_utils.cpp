#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"

#include <yaml-cpp/yaml.h>

namespace legged_robot_mpc_controller::common
{

ocs2::humanoid::GaitMap loadGaitMap(const std::string& gaitFile)
{
  const YAML::Node root = YAML::LoadFile(gaitFile);
  if (!root["list"]) {
    throw std::runtime_error("[config_builder] gait file has no 'list' key: " + gaitFile);
  }

  ocs2::humanoid::GaitMap gaitMap;
  for (const auto& gaitNameNode : root["list"]) {
    const auto gaitName = gaitNameNode.as<std::string>();
    const YAML::Node gait = root[gaitName];
    if (!gait || !gait["modeSequence"] || !gait["switchingTimes"]) {
      throw std::runtime_error("[config_builder] gait '" + gaitName + "' is missing in " + gaitFile);
    }
    const auto modeSequence = gait["modeSequence"].as<std::vector<std::string>>();
    const auto switchingTimes = gait["switchingTimes"].as<std::vector<double>>();
    gaitMap.insert({gaitName, ocs2::humanoid::modeSequenceTemplateFromStrings(switchingTimes, modeSequence)});
  }
  return gaitMap;
}

}  // namespace legged_robot_mpc_controller::common
