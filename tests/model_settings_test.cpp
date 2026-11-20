#include <pinocchio/fwd.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h"
#include "legged_robot_mpc_controller/mpc/common/model/createPinocchioModel.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Function>
void requireInvalid(Function function)
{
  try {
    function();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("invalid fixed-joint configuration was accepted");
}

void checkRobot(const std::filesystem::path & root, const std::string & name, size_t fullJointCount)
{
  ModelSettings::Params params;
  params.robotName = name;
  params.recompileLibrariesCppAd = false;
  params.verboseCppAd = false;
  std::string urdfName;
  if (name == "upkie") {
    urdfName = "upkie.urdf";
    params.contactNames6DoF = {"left_tire_center", "right_tire_center"};
    params.contactParentJointNames = {"left_wheel", "right_wheel"};
  } else {
    urdfName = name == "g1" ? "g1_29dof.urdf" : "h2_31dof.urdf";
    params.fixedJointNames = {"left_wrist_roll_joint", "left_wrist_pitch_joint",
      "left_wrist_yaw_joint",
      "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
    params.contactNames6DoF = {"foot_l_contact", "foot_r_contact"};
    params.contactParentJointNames = name == "g1" ?
      std::vector<std::string>{"left_ankle_roll_joint", "right_ankle_roll_joint"} :
    std::vector<std::string>{"left_ankle_pitch_joint", "right_ankle_pitch_joint"};
  }
  const std::string urdf = (root / name / urdfName).string();
  const ModelSettings settings(params, urdf, "model_settings_test_");
  require(settings.full_joint_dim == fullJointCount, name + ": unexpected full joint count");
  require(settings.mpc_joint_dim == fullJointCount - params.fixedJointNames.size(),
          name + ": unexpected active joint count");
  require(settings.mpcModelToFullJointsIndices.size() == settings.mpc_joint_dim,
          name + ": active-to-full joint index vector has the wrong size");
  for (size_t i = 0; i < settings.mpc_joint_dim; ++i) {
    const size_t fullIndex = settings.mpcModelToFullJointsIndices.at(i);
    require(fullIndex < settings.full_joint_dim, name + ": joint index is out of bounds");
    require(settings.fullJointNames.at(fullIndex) == settings.mpcModelJointNames.at(i),
            name + ": active joint maps to the wrong full joint");
  }

  const WbIdMpcRobotModel<scalar_t> robot(settings);
  const vector_t full = vector_t::LinSpaced(settings.full_joint_dim, -0.5, 0.5);
  const vector_t active = robot.getMpcModelJointAngles(full);
  const vector_t defaults = vector_t::Constant(settings.full_joint_dim, 0.75);
  const vector_t restored = robot.getFullModelJointAngles(active, defaults);
  for (size_t i = 0; i < settings.full_joint_dim; ++i) {
    const bool isActive = settings.jointIndexMap.count(settings.fullJointNames.at(i)) != 0;
    require(restored(i) == (isActive ? full(i) : defaults(i)),
        name + ": joint-angle mapping changed a value");
  }
  require(robot.getMpcModelJointAngles(restored).isApprox(active),
      name + ": joint-angle round trip failed");

  const auto interface = createCustomPinocchioInterface(urdf, settings);
  const auto contacts = getContactFrameIndices(interface, robot);
  require(contacts.size() == N_CONTACTS, name + ": contact index vector has the wrong size");
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    require(contacts.at(i) == interface.getModel().getFrameId(settings.contactNames.at(i)),
            name + ": contact index does not match its frame");
  }

  auto invalid = params;
  invalid.fixedJointNames.push_back("not_a_robot_joint");
  requireInvalid([&] {ModelSettings rejected(invalid, urdf, "invalid_");});
  invalid.fixedJointNames = {settings.fullJointNames.front(), settings.fullJointNames.front()};
  requireInvalid([&] {ModelSettings rejected(invalid, urdf, "invalid_");});
  invalid.fixedJointNames = settings.fullJointNames;
  requireInvalid([&] {ModelSettings rejected(invalid, urdf, "invalid_");});
  std::cout << name <<
    ": joint mapping, contact indices, and invalid configuration checks passed\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "usage: model_settings_test ROBOT_URDF_ROOT");
    checkRobot(argv[1], "g1", 29);
    checkRobot(argv[1], "h2", 31);
    checkRobot(argv[1], "upkie", 6);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
