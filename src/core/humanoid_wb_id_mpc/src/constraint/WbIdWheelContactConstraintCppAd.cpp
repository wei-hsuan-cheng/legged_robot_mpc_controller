#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/constraint/WbIdWheelContactConstraintCppAd.h"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_wb_id_mpc/common/WheelContactKinematics.h"

namespace ocs2::humanoid {

namespace {

// Parameter layout, shared by all three types so one vector serves every term:
//   0 ka_t, 1 kd_t, 2 ka_n, 3 kd_n, 4 kp_n, 5 radius, 6-8 axle (contact frame), 9-11 normal (world),
//   12 z_ref, 13 vz_ref, 14 az_ref, 15.. contact weights w_i (Lateral)
constexpr size_t kGainKaT = 0;
constexpr size_t kGainKdT = 1;
constexpr size_t kGainKaN = 2;
constexpr size_t kGainKdN = 3;
constexpr size_t kGainKpN = 4;
constexpr size_t kRadius = 5;
constexpr size_t kAxle = 6;
constexpr size_t kNormal = 9;
constexpr size_t kZRef = 12;
constexpr size_t kVzRef = 13;
constexpr size_t kAzRef = 14;
constexpr size_t kWeights = 15;
constexpr size_t kNumParameters = kWeights + N_CONTACTS;

std::string typeTag(WbIdWheelContactConstraintCppAd::Type type) {
  switch (type) {
    case WbIdWheelContactConstraintCppAd::Type::Rolling:
      return "rolling";
    case WbIdWheelContactConstraintCppAd::Type::Lateral:
      return "lateral";
    case WbIdWheelContactConstraintCppAd::Type::SwingVertical:
      return "swing_vertical";
  }
  return "";
}

}  // namespace

WbIdWheelContactConstraintCppAd::WbIdWheelContactConstraintCppAd(Type type,
                                                                 const SwitchedModelReferenceManager& referenceManager,
                                                                 const PinocchioInterface& pinocchioInterface,
                                                                 const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                                                 size_t contactIndex,
                                                                 Settings settings,
                                                                 const ModelSettings& modelSettings)
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      type_(type),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr_(mpcRobotModelAD.clone()),
      contactIndex_(contactIndex),
      settings_(std::move(settings)) {
  const auto& model = pinocchioInterface.getModel();
  const auto frameOf = [&](size_t index) {
    const std::string& name = modelSettings.contactNames.at(index);
    const auto frameId = model.getFrameId(name);
    if (frameId >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
      throw std::invalid_argument("[WbIdWheelContactConstraintCppAd] contact frame '" + name + "' not in the pinocchio model");
    }
    return static_cast<size_t>(frameId);
  };
  if (type_ == Type::Lateral) {
    for (size_t i = 0; i < modelSettings.contactNames.size(); ++i) {
      frameIds_.push_back(frameOf(i));
    }
  } else {
    frameIds_.push_back(frameOf(contactIndex_));
  }

  if (!std::isfinite(settings_.radius) || settings_.radius <= 0.0) {
    throw std::invalid_argument("[WbIdWheelContactConstraintCppAd] wheel radius must be positive");
  }
  if (!settings_.axleAxis.allFinite() || settings_.axleAxis.norm() < 1e-6) {
    throw std::invalid_argument("[WbIdWheelContactConstraintCppAd] the axle axis must be a nonzero vector");
  }
  settings_.axleAxis.normalize();
  // Without the acceleration terms the rows have no input dependence and the state-input
  // projection is singular; refuse that rather than fail inside the solver.
  if (settings_.normalAccelerationGain == 0.0 || settings_.tangentialAccelerationGain == 0.0) {
    throw std::invalid_argument("[WbIdWheelContactConstraintCppAd] wheel contact acceleration gains must be non-zero");
  }

  const std::string prefix = type_ == Type::Lateral ? std::string("wheels") : modelSettings.contactNames.at(contactIndex_);
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), kNumParameters,
             prefix + "_wb_id_wheel_" + typeTag(type_) + "_v1", modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

WbIdWheelContactConstraintCppAd::WbIdWheelContactConstraintCppAd(const WbIdWheelContactConstraintCppAd& rhs)
    : StateInputConstraintCppAd(rhs),
      type_(rhs.type_),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(rhs.pinocchioInterfaceCppAd_),
      mpcRobotModelADPtr_(rhs.mpcRobotModelADPtr_->clone()),
      contactIndex_(rhs.contactIndex_),
      frameIds_(rhs.frameIds_),
      settings_(rhs.settings_) {}

bool WbIdWheelContactConstraintCppAd::isActive(scalar_t time) const {
  const auto flags = referenceManagerPtr_->getContactFlags(time);
  switch (type_) {
    case Type::Rolling:
      return flags[contactIndex_];
    case Type::SwingVertical:
      return !flags[contactIndex_];
    case Type::Lateral:
      for (const bool flag : flags) {
        if (flag) {
          return true;
        }
      }
      return false;
  }
  return false;
}

size_t WbIdWheelContactConstraintCppAd::getNumConstraints(scalar_t /*time*/) const {
  return type_ == Type::Rolling ? 5 : 1;
}

vector_t WbIdWheelContactConstraintCppAd::getParameters(scalar_t time, const PreComputation& /*preComputation*/) const {
  vector_t p = vector_t::Zero(kNumParameters);
  p(kGainKaT) = settings_.tangentialAccelerationGain;
  p(kGainKdT) = settings_.tangentialVelocityGain;
  p(kGainKaN) = settings_.normalAccelerationGain;
  p(kGainKdN) = settings_.normalVelocityGain;
  p(kGainKpN) = settings_.normalPositionGain;
  p(kRadius) = settings_.radius;
  p.segment<3>(kAxle) = settings_.axleAxis;

  const auto flags = referenceManagerPtr_->getContactFlags(time);
  if (type_ == Type::Lateral) {
    // The plane of the first wheel in contact; on flat ground every wheel shares it.
    size_t numInContact = 0;
    vector3_t normal(0.0, 0.0, 1.0);
    for (size_t i = 0; i < flags.size(); ++i) {
      if (flags[i]) {
        if (numInContact == 0) {
          normal = referenceManagerPtr_->getContactNormal(i, time);
        }
        ++numInContact;
      }
    }
    p.segment<3>(kNormal) = normal;
    for (size_t i = 0; i < flags.size() && i < N_CONTACTS; ++i) {
      p(kWeights + i) = (flags[i] && numInContact > 0) ? 1.0 / static_cast<scalar_t>(numInContact) : 0.0;
    }
    return p;
  }

  const auto& swing = *referenceManagerPtr_->getSwingTrajectoryPlanner();
  p.segment<3>(kNormal) = referenceManagerPtr_->getContactNormal(contactIndex_, time);
  p(kZRef) = swing.getZpositionConstraint(contactIndex_, time);
  p(kVzRef) = swing.getZvelocityConstraint(contactIndex_, time);
  p(kAzRef) = swing.getZaccelerationConstraint(contactIndex_, time);
  return p;
}

ad_vector_t WbIdWheelContactConstraintCppAd::constraintFunction(ad_scalar_t /*time*/,
                                                                const ad_vector_t& state,
                                                                const ad_vector_t& input,
                                                                const ad_vector_t& p) const {
  using ad_vector3 = Eigen::Matrix<ad_scalar_t, 3, 1>;
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();  // const method: record on a copy

  const ad_vector_t q = mpcRobotModelADPtr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelADPtr_->getGeneralizedVelocities(state);
  const ad_vector_t a = mpcRobotModelADPtr_->getGeneralizedAccelerations(input);
  pinocchio::forwardKinematics(model, data, q, v, a);

  const ad_scalar_t& kaT = p[kGainKaT];
  const ad_scalar_t& kdT = p[kGainKdT];
  const ad_scalar_t& kaN = p[kGainKaN];
  const ad_scalar_t& kdN = p[kGainKdN];
  const ad_scalar_t& kpN = p[kGainKpN];
  const ad_scalar_t& radius = p[kRadius];
  const ad_vector3 axleLocal = p.segment<3>(kAxle);
  const ad_vector3 normal = p.segment<3>(kNormal);

  struct WheelKinematics {
    ad_vector3 centre, rho, vContact, aContact, axle, vCentre, aCentre;
  };
  const auto wheelKinematics = [&](size_t frameId) {
    WheelKinematics k;
    const auto& placement = pinocchio::updateFramePlacement(model, data, frameId);
    const auto twist = pinocchio::getFrameVelocity(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED);
    const auto acceleration = pinocchio::getFrameClassicalAcceleration(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED);
    const ad_vector3 omega = twist.angular();
    const ad_vector3 alpha = acceleration.angular();
    k.centre = placement.translation();
    k.vCentre = twist.linear();
    k.aCentre = acceleration.linear();
    k.axle = placement.rotation() * axleLocal;
    k.rho = wheel::contactOffset<ad_scalar_t>(k.axle, normal, radius);
    const ad_vector3 rhoRate = wheel::contactOffsetRate<ad_scalar_t>(k.axle, omega.cross(k.axle), normal, radius);
    k.vContact = k.vCentre + omega.cross(k.rho);
    k.aContact = k.aCentre + alpha.cross(k.rho) + omega.cross(rhoRate);
    return k;
  };

  if (type_ == Type::Lateral) {
    ad_vector_t g = ad_vector_t::Zero(1);
    for (size_t i = 0; i < frameIds_.size() && i < N_CONTACTS; ++i) {
      const WheelKinematics k = wheelKinematics(frameIds_[i]);
      const ad_vector3 lateral = wheel::lateralDirection<ad_scalar_t>(k.axle, normal);
      g(0) += p[kWeights + i] * (kaT * lateral.dot(k.aContact) + kdT * lateral.dot(k.vContact));
    }
    return g;
  }

  const WheelKinematics k = wheelKinematics(frameIds_.front());
  if (type_ == Type::SwingVertical) {
    // The tire bottom of a level wheel, z_h - r, against the swing planner's height reference.
    ad_vector_t g(1);
    g(0) = kaN * (k.aCentre(2) - p[kAzRef]) + kdN * (k.vCentre(2) - p[kVzRef]) + kpN * (k.centre(2) - radius - p[kZRef]);
    return g;
  }

  const ad_vector3 longitudinal = wheel::longitudinalDirection<ad_scalar_t>(k.axle, normal);
  const ad_vector3 contactPoint = k.centre + k.rho;
  const auto wrench = mpcRobotModelADPtr_->getContactWrench(input, contactIndex_);
  const ad_vector3 force = wrench.template head<3>();
  const ad_vector3 moment = wrench.template tail<3>();

  ad_vector_t g(5);
  g(0) = kaT * longitudinal.dot(k.aContact) + kdT * longitudinal.dot(k.vContact);
  g(1) = kaN * normal.dot(k.aContact) + kdN * normal.dot(k.vContact) + kpN * (normal.dot(contactPoint) - p[kZRef]);
  g.tail<3>() = moment - k.rho.cross(force);
  return g;
}

}  // namespace ocs2::humanoid
