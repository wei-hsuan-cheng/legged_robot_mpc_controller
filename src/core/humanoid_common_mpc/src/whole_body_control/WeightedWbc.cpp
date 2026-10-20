/******************************************************************************
Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

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
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/

#include <pinocchio/fwd.hpp>  // must precede every other pinocchio header

#include "humanoid_common_mpc/whole_body_control/WeightedWbc.h"

#include <cmath>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "humanoid_common_mpc/contact/ContactRectangle.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"

namespace ocs2::humanoid {

WeightedWbc::WeightedWbc(PinocchioInterface pinocchioInterface,
                         const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                         const ModelSettings& modelSettings,
                         WbcSettings settings)
    : pinocchioMeasured_(pinocchioInterface),
      pinocchioDesired_(std::move(pinocchioInterface)),
      mpcRobotModel_(mpcRobotModel.clone()),
      settings_(std::move(settings)),
      qp_(std::make_unique<HpipmDenseQp>()) {
  const auto& model = pinocchioMeasured_.getModel();
  nv_ = static_cast<size_t>(model.nv);
  nj_ = mpcRobotModel_->getJointDim();
  numDecisionVars_ = nv_ + 6 * N_CONTACTS + nj_;

  footBounds_.reserve(N_CONTACTS);
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    contactFrameIds_[i] = model.getFrameId(modelSettings.contactNames6DoF[i]);
    footBounds_.push_back(ContactRectangle::fromModelSettings(modelSettings, static_cast<int>(i)).getBounds());
  }

  qMeasured_ = vector_t::Zero(model.nq);
  vMeasured_ = vector_t::Zero(model.nv);
}

WeightedWbc::~WeightedWbc() = default;

/******************************************************************************************************/

void WeightedWbc::updateMeasured(const vector_t& qMeasured, const vector_t& vMeasured) {
  qMeasured_ = qMeasured;
  vMeasured_ = vMeasured;

  const auto& model = pinocchioMeasured_.getModel();
  auto& data = pinocchioMeasured_.getData();

  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  // crba only fills the upper triangle.
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);

  j_.setZero(6 * N_CONTACTS, nv_);
  dj_.setZero(6 * N_CONTACTS, nv_);
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    matrix_t jac = matrix_t::Zero(6, nv_);
    matrix_t djac = matrix_t::Zero(6, nv_);
    // LOCAL_WORLD_ALIGNED matches the convention the MPC's contact wrenches and
    // the previous closed-form inverse dynamics already use.
    pinocchio::getFrameJacobian(model, data, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    pinocchio::getFrameJacobianTimeVariation(model, data, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, djac);
    j_.middleRows(6 * i, 6) = jac;
    dj_.middleRows(6 * i, 6) = djac;
  }
}

/******************************************************************************************************/

void WeightedWbc::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) {
  qDesired_ = mpcRobotModel_->getGeneralizedCoordinates(stateDesired);
  vDesired_ = mpcRobotModel_->getGeneralizedVelocities(stateDesired, inputDesired);

  const auto& model = pinocchioDesired_.getModel();
  auto& data = pinocchioDesired_.getData();
  pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);
  pinocchio::updateFramePlacements(model, data);

  for (size_t i = 0; i < N_CONTACTS; ++i) {
    const auto& placement = data.oMf[contactFrameIds_[i]];
    desiredFootPosition_[i] = placement.translation();
    desiredFootOrientation_[i] = placement.rotation();
    desiredFootVelocity_[i] =
        pinocchio::getFrameVelocity(model, data, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
    desiredContactWrench_[i] = mpcRobotModel_->getContactWrench(inputDesired, i);
  }
}

/******************************************************************************************************/
/*                                       hard constraints                                             */
/******************************************************************************************************/

Task WeightedWbc::formulateFloatingBaseEomTask() const {
  const auto& data = pinocchioMeasured_.getData();

  // M qddot + nle = J^T w + S^T tau, with S selecting the actuated joints.
  matrix_t s = matrix_t::Zero(nj_, nv_);
  s.rightCols(nj_).setIdentity();

  matrix_t a = matrix_t::Zero(nv_, numDecisionVars_);
  a.leftCols(nv_) = data.M;
  a.middleCols(nv_, 6 * N_CONTACTS) = -j_.transpose();
  a.rightCols(nj_) = -s.transpose();

  return {a, vector_t(-data.nle), matrix_t(), vector_t()};
}

Task WeightedWbc::formulateNoContactMotionTask() const {
  // A foot in stance does not accelerate: J qddot + Jdot v = 0, all six rows,
  // because a flat foot must not rotate either.
  matrix_t a = matrix_t::Zero(6 * numStanceContacts_, numDecisionVars_);
  vector_t b = vector_t::Zero(a.rows());

  size_t row = 0;
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    if (contactFlags_[i]) {
      a.block(6 * row, 0, 6, nv_) = j_.middleRows(6 * i, 6);
      b.segment(6 * row, 6) = -dj_.middleRows(6 * i, 6) * vMeasured_;
      ++row;
    }
  }
  return {a, b, matrix_t(), vector_t()};
}

Task WeightedWbc::formulateZeroSwingWrenchTask() const {
  const size_t numSwing = N_CONTACTS - numStanceContacts_;
  matrix_t a = matrix_t::Zero(6 * numSwing, numDecisionVars_);
  vector_t b = vector_t::Zero(a.rows());

  size_t row = 0;
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    if (!contactFlags_[i]) {
      a.block(6 * row, wrenchStartIndex(i), 6, 6).setIdentity();
      ++row;
    }
  }
  return {a, b, matrix_t(), vector_t()};
}

Task WeightedWbc::formulateContactWrenchConeTask() const {
  // Per stance foot, with f = w.head(3) and m = w.tail(3) in LOCAL_WORLD_ALIGNED:
  //   1 row   unilateral        -f_z <= -f_min
  //   4 rows  friction pyramid  +/-f_x - mu f_z <= 0, +/-f_y - mu f_z <= 0
  //   4 rows  centre of pressure inside the foot rectangle
  //   2 rows  torsional friction +/-m_z - mu_t f_z <= 0
  constexpr size_t kRowsPerContact = 11;
  matrix_t d = matrix_t::Zero(kRowsPerContact * numStanceContacts_, numDecisionVars_);
  vector_t f = vector_t::Zero(d.rows());

  const scalar_t mu = settings_.frictionCoefficient;
  const scalar_t muT = settings_.torsionalFrictionCoefficient;

  size_t row = 0;
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    if (!contactFlags_[i]) {
      continue;
    }
    const auto& bounds = footBounds_[i];
    const scalar_t xMin = bounds.x_min * settings_.copMarginScale;
    const scalar_t xMax = bounds.x_max * settings_.copMarginScale;
    const scalar_t yMin = bounds.y_min * settings_.copMarginScale;
    const scalar_t yMax = bounds.y_max * settings_.copMarginScale;

    const size_t fx = wrenchStartIndex(i) + 0;
    const size_t fy = wrenchStartIndex(i) + 1;
    const size_t fz = wrenchStartIndex(i) + 2;
    const size_t mx = wrenchStartIndex(i) + 3;
    const size_t my = wrenchStartIndex(i) + 4;
    const size_t mz = wrenchStartIndex(i) + 5;

    size_t r = kRowsPerContact * row;
    d(r, fz) = -1.0;
    f(r) = -settings_.minContactForceZ;
    ++r;

    d(r, fx) = 1.0;  d(r, fz) = -mu; ++r;
    d(r, fx) = -1.0; d(r, fz) = -mu; ++r;
    d(r, fy) = 1.0;  d(r, fz) = -mu; ++r;
    d(r, fy) = -1.0; d(r, fz) = -mu; ++r;

    // CoP_x = -m_y / f_z in [xMin, xMax], CoP_y = m_x / f_z in [yMin, yMax].
    d(r, my) = -1.0; d(r, fz) = -xMax; ++r;
    d(r, my) = 1.0;  d(r, fz) = xMin;  ++r;
    d(r, mx) = 1.0;  d(r, fz) = -yMax; ++r;
    d(r, mx) = -1.0; d(r, fz) = yMin;  ++r;

    d(r, mz) = 1.0;  d(r, fz) = -muT; ++r;
    d(r, mz) = -1.0; d(r, fz) = -muT; ++r;

    ++row;
  }
  return {matrix_t(), vector_t(), d, f};
}

/******************************************************************************************************/
/*                                      weighted objectives                                           */
/******************************************************************************************************/

Task WeightedWbc::formulateBaseAccelTask() const {
  matrix_t a = matrix_t::Zero(6, numDecisionVars_);
  a.topLeftCorner(6, 6).setIdentity();

  // The base states are [position, euler ZYX] in this model, so a plain
  // difference is the correct error for both halves at small angles.
  const vector6_t poseError = qDesired_.head(6) - qMeasured_.head(6);
  const vector6_t twistError = vDesired_.head(6) - vMeasured_.head(6);
  const vector6_t b = settings_.baseKp * poseError + settings_.baseKd * twistError;

  return {a, b, matrix_t(), vector_t()};
}

Task WeightedWbc::formulateJointAccelTask() const {
  matrix_t a = matrix_t::Zero(nj_, numDecisionVars_);
  a.block(0, 6, nj_, nj_).setIdentity();

  const vector_t positionError = qDesired_.tail(nj_) - qMeasured_.tail(nj_);
  const vector_t velocityError = vDesired_.tail(nj_) - vMeasured_.tail(nj_);
  const vector_t b = settings_.jointKp * positionError + settings_.jointKd * velocityError;

  return {a, b, matrix_t(), vector_t()};
}

Task WeightedWbc::formulateSwingFootTask() const {
  const size_t numSwing = N_CONTACTS - numStanceContacts_;
  matrix_t a = matrix_t::Zero(6 * numSwing, numDecisionVars_);
  vector_t b = vector_t::Zero(a.rows());

  const auto& model = pinocchioMeasured_.getModel();
  const auto& data = pinocchioMeasured_.getData();

  size_t row = 0;
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    if (contactFlags_[i]) {
      continue;
    }
    const auto& placement = data.oMf[contactFrameIds_[i]];
    const vector6_t velocity =
        pinocchio::getFrameVelocity(model, data, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED).toVector();

    vector6_t poseError;
    poseError.head(3) = desiredFootPosition_[i] - placement.translation();
    // Orientation error as a world-frame rotation vector.
    poseError.tail(3) = pinocchio::log3(desiredFootOrientation_[i] * placement.rotation().transpose());

    const vector6_t accel =
        settings_.swingKp * poseError + settings_.swingKd * (desiredFootVelocity_[i] - velocity);

    a.block(6 * row, 0, 6, nv_) = j_.middleRows(6 * i, 6);
    b.segment(6 * row, 6) = accel - dj_.middleRows(6 * i, 6) * vMeasured_;
    ++row;
  }
  return {a, b, matrix_t(), vector_t()};
}

Task WeightedWbc::formulateContactWrenchTask() const {
  // Pull the stance wrenches toward what the MPC optimised. Swing feet are
  // already pinned to zero by an equality, so they are left out here.
  matrix_t a = matrix_t::Zero(6 * numStanceContacts_, numDecisionVars_);
  vector_t b = vector_t::Zero(a.rows());

  size_t row = 0;
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    if (contactFlags_[i]) {
      a.block(6 * row, wrenchStartIndex(i), 6, 6).setIdentity();
      b.segment(6 * row, 6) = desiredContactWrench_[i];
      ++row;
    }
  }
  return {a, b, matrix_t(), vector_t()};
}

Task WeightedWbc::formulateRegularisationTask() const {
  // Keeps the Hessian positive definite and resolves whatever redundancy the
  // other tasks leave, instead of letting the solver pick an arbitrary point.
  matrix_t a = matrix_t::Zero(nj_ + nv_, numDecisionVars_);
  a.block(0, torqueStartIndex(), nj_, nj_) = std::sqrt(settings_.weightTorqueReg) * matrix_t::Identity(nj_, nj_);
  a.block(nj_, 0, nv_, nv_) = std::sqrt(settings_.weightAccelReg) * matrix_t::Identity(nv_, nv_);
  return {a, vector_t::Zero(a.rows()), matrix_t(), vector_t()};
}

/******************************************************************************************************/

WeightedWbc::Solution WeightedWbc::update(const vector_t& stateDesired,
                                          const vector_t& inputDesired,
                                          const vector_t& qMeasured,
                                          const vector_t& vMeasured,
                                          size_t mode) {
  contactFlags_ = modeNumber2StanceLeg(mode);
  numStanceContacts_ = 0;
  for (bool flag : contactFlags_) {
    if (flag) {
      ++numStanceContacts_;
    }
  }

  updateMeasured(qMeasured, vMeasured);
  updateDesired(stateDesired, inputDesired);

  // Equalities and inequalities are hard; the rest is least squares.
  const Task constraints =
      formulateFloatingBaseEomTask() + formulateNoContactMotionTask() + formulateZeroSwingWrenchTask();
  const Task cone = formulateContactWrenchConeTask();

  // sqrt weights, so that stacking the rows reproduces the weighted sum of squares.
  const Task objective = formulateBaseAccelTask() * std::sqrt(settings_.weightBaseAccel) +
                         formulateJointAccelTask() * std::sqrt(settings_.weightJointAccel) +
                         formulateSwingFootTask() * std::sqrt(settings_.weightSwing) +
                         formulateContactWrenchTask() * std::sqrt(settings_.weightContactWrench) +
                         formulateRegularisationTask();

  const matrix_t H = objective.a_.transpose() * objective.a_;
  const vector_t g = -objective.a_.transpose() * objective.b_;

  // Only the torques carry a bound; everything else is left unbounded by simply
  // not listing it, which is what HPIPM wants.
  std::vector<int> boundedIndices;
  vector_t lowerBound, upperBound;
  if (settings_.torqueLimits.size() == static_cast<Eigen::Index>(nj_)) {
    boundedIndices.resize(nj_);
    lowerBound.resize(nj_);
    upperBound.resize(nj_);
    for (size_t i = 0; i < nj_; ++i) {
      boundedIndices[i] = static_cast<int>(torqueStartIndex() + i);
      lowerBound(static_cast<Eigen::Index>(i)) = -settings_.torqueLimits(static_cast<Eigen::Index>(i));
      upperBound(static_cast<Eigen::Index>(i)) = settings_.torqueLimits(static_cast<Eigen::Index>(i));
    }
  }

  Solution solution;
  vector_t x;
  solution.success =
      qp_->solve(H, g, constraints.a_, constraints.b_, cone.d_, cone.f_, boundedIndices, lowerBound, upperBound, x);
  solution.iterations = qp_->lastIterationCount();

  if (!solution.success || x.size() != static_cast<Eigen::Index>(numDecisionVars_)) {
    solution.jointTorques = vector_t::Zero(nj_);
    solution.jointAccelerations = vector_t::Zero(nj_);
    for (size_t i = 0; i < N_CONTACTS; ++i) {
      solution.contactWrenches[i].setZero();
    }
    return solution;
  }

  solution.jointAccelerations = x.segment(6, nj_);
  solution.jointTorques = x.tail(nj_);
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    solution.contactWrenches[i] = x.segment(wrenchStartIndex(i), 6);
  }
  return solution;
}

}  // namespace ocs2::humanoid
