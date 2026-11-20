/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "legged_robot_mpc_controller/mpc/common/model/ModelSettings.h"

#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "legged_robot_mpc_controller/mpc/common/model/createPinocchioModel.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/// Helper functions contained in a local anonymous namespace
/******************************************************************************************************/
namespace {

/**
 * @brief Creates a joint Index map from a list of joint names.
 */

static std::unordered_map<std::string, size_t> createJointIndexMap(const std::vector<std::string>& jointNames, size_t offset = 0) {
  std::unordered_map<std::string, size_t> jointIndexMap;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointIndexMap[jointNames[i]] = i + offset;
  }
  return jointIndexMap;
}

static std::vector<std::string> initializeJointNames(const std::vector<std::string>& fullJointNames,
                                                     const std::vector<std::string>& fixedJointNames,
                                                     bool verbose) {
  std::unordered_set<std::string> fixedJoints;
  for (const auto& joint : fixedJointNames) {
    if (std::find(fullJointNames.begin(), fullJointNames.end(), joint) == fullJointNames.end()) {
      throw std::invalid_argument("[ModelSettings] unknown fixed joint: " + joint);
    }
    if (!fixedJoints.insert(joint).second) {
      throw std::invalid_argument("[ModelSettings] duplicate fixed joint: " + joint);
    }
  }
  if (fixedJoints.size() >= fullJointNames.size()) {
    throw std::invalid_argument("Number of joints must be greater than zero");
  }
  if (verbose) std::cout << "Initialize the following active MPC joints: " << std::endl;
  const size_t n_joints = fullJointNames.size() - fixedJoints.size();
  if (verbose) std::cout << "Num active joints: " << n_joints << std::endl;
  std::vector<std::string> mpcModelJointNames;
  mpcModelJointNames.reserve(n_joints);
  for (const auto& joint : fullJointNames) {
    if (fixedJoints.count(joint) == 0) {
      // If the joint is not found in fixedJointNames, add it to mpcModelJointNames
      if (verbose) std::cout << joint << std::endl;
      mpcModelJointNames.emplace_back(joint);
    }
  }
  return mpcModelJointNames;
}

std::vector<size_t> initializeMpcToFullJointIndices(const std::vector<std::string>& fullJointNames,
                                                    const std::vector<std::string>& mpcModelJointNames) {
  const auto fullJointIndexMap = createJointIndexMap(fullJointNames);
  std::vector<size_t> mpcModelJointIndices(mpcModelJointNames.size());
  for (size_t i = 0; i < mpcModelJointNames.size(); ++i) {
    mpcModelJointIndices[i] = fullJointIndexMap.at(mpcModelJointNames[i]);
  }
  return mpcModelJointIndices;
}

std::vector<std::string> concatenateStringVectors(const std::vector<std::string>& a, const std::vector<std::string>& b) {
  std::vector<std::string> temp_vec(a);
  temp_vec.insert(temp_vec.begin(), b.begin(), b.end());
  return temp_vec;
}

}  // namespace

ModelSettings::ModelSettings(const Params& params, const std::string& urdfFile, const std::string& mpcName, bool verbose) {
  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  this->robotName = params.robotName;
  this->verboseCppAd = params.verboseCppAd;
  this->recompileLibrariesCppAd = params.recompileLibrariesCppAd;
  this->phaseTransitionStanceTime = params.phaseTransitionStanceTime;

  this->j_l_shoulder_y_name = params.j_l_shoulder_y_name;
  this->j_r_shoulder_y_name = params.j_r_shoulder_y_name;
  this->j_l_elbow_y_name = params.j_l_elbow_y_name;
  this->j_r_elbow_y_name = params.j_r_elbow_y_name;
  if (params.modelFolderCppAd.empty()) {
    modelFolderCppAd = "cppad_code_gen/cppad_" + mpcName + robotName;
  } else {
    modelFolderCppAd = params.modelFolderCppAd + "/" + mpcName + robotName;
  }

  this->fixedJointNames = params.fixedJointNames;
  this->contactNames6DoF = params.contactNames6DoF;
  this->contactParentJointNames = params.contactParentJointNames;
  this->footConstraintConfig = params.footConstraintConfig;

  this->contactFrameTranslation = params.contactFrameTranslation;
  this->contactRectangleXMin = params.contactRectangleXMin;
  this->contactRectangleXMax = params.contactRectangleXMax;
  this->contactRectangleYMin = params.contactRectangleYMin;
  this->contactRectangleYMax = params.contactRectangleYMax;
  this->contactRectangleScaleFactor = params.contactRectangleScaleFactor;

  if (verbose) {
    std::cout << "Initializing MPC by fixing joints: " << std::endl;
    for (const auto& fixedJoint : fixedJointNames) std::cout << fixedJoint << std::endl;
  }

  // Get full joint order from a full pinocchio interface, this removes any joints marked as fix in the urdf.
  PinocchioInterface fullPinocchioInterface = createDefaultPinocchioInterface(urdfFile);
  const pinocchio::Model& model = fullPinocchioInterface.getModel();
  if (verbose) std::cout << "Full URDF joints: " << std::endl;
  fullJointNames.reserve(model.njoints - 2);  // Substract universe and root joint
  for (pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) {
    if (verbose) std::cout << model.names[joint_id] << std::endl;
    fullJointNames.emplace_back(model.names[joint_id]);
  }

  this->mpcModelJointNames = initializeJointNames(this->fullJointNames, this->fixedJointNames, verbose);
  this->mpcModelToFullJointsIndices = initializeMpcToFullJointIndices(this->fullJointNames, this->mpcModelJointNames);
  this->jointIndexMap = createJointIndexMap(this->mpcModelJointNames);
  this->contactNames = concatenateStringVectors(this->contactNames3DoF, this->contactNames6DoF);

  this->mpc_joint_dim = this->mpcModelJointNames.size();
  this->full_joint_dim = this->fullJointNames.size();

  // A robot without arms (e.g. a wheeled-biped) leaves all four arm-swing names empty.
  const bool noArmNames = j_l_shoulder_y_name.empty() && j_r_shoulder_y_name.empty() && j_l_elbow_y_name.empty() &&
                          j_r_elbow_y_name.empty();
  hasArmSwingJoints = !noArmNames;
  if (hasArmSwingJoints) {
    j_l_shoulder_y_index = this->jointIndexMap.at(j_l_shoulder_y_name);
    j_r_shoulder_y_index = this->jointIndexMap.at(j_r_shoulder_y_name);
    j_l_elbow_y_index = this->jointIndexMap.at(j_l_elbow_y_name);
    j_r_elbow_y_index = this->jointIndexMap.at(j_r_elbow_y_name);
  }

  this->jointArmature = params.jointArmature;
  if (!jointArmature.empty() && jointArmature.size() != mpc_joint_dim) {
    throw std::invalid_argument("[ModelSettings] jointArmature needs one entry per MPC joint (" + std::to_string(mpc_joint_dim) +
                                "), got " + std::to_string(jointArmature.size()));
  }
  for (const scalar_t armature : jointArmature) {
    if (!std::isfinite(armature) || armature < 0.0) {
      throw std::invalid_argument("[ModelSettings] jointArmature entries must be finite and non-negative");
    }
  }

  this->jointFriction = params.jointFriction;
  this->jointFrictionVelocity = params.jointFrictionVelocity;
  if (!jointFriction.empty() && jointFriction.size() != mpc_joint_dim) {
    throw std::invalid_argument("[ModelSettings] jointFriction needs one entry per MPC joint (" + std::to_string(mpc_joint_dim) +
                                "), got " + std::to_string(jointFriction.size()));
  }
  for (const scalar_t friction : jointFriction) {
    if (!std::isfinite(friction) || friction < 0.0) {
      throw std::invalid_argument("[ModelSettings] jointFriction entries must be finite and non-negative");
    }
  }
  if (!jointFriction.empty() && !(std::isfinite(jointFrictionVelocity) && jointFrictionVelocity > 0.0)) {
    throw std::invalid_argument("[ModelSettings] jointFrictionVelocity must be positive");
  }

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2::humanoid
