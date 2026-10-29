#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h>
#include <humanoid_wb_id_mpc/common/WbIdPinocchioMapping.h>
#include <humanoid_wb_id_mpc/dynamics/WbIdTangentDynamics.h>
#include <humanoid_wb_id_mpc/cost/WbIdBaseMotionCost.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <iostream>
#include <stdexcept>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  const auto check = [](bool ok, const char* message) { if (!ok) throw std::runtime_error(message); };
  ModelSettings::Params params;
  params.robotName = "h2";
  params.verboseCppAd = false;
  params.recompileLibrariesCppAd = true;
  params.modelFolderCppAd = "/tmp/wb_id_tangent_native";
  params.j_l_shoulder_y_name = "left_shoulder_pitch_joint";
  params.j_r_shoulder_y_name = "right_shoulder_pitch_joint";
  params.j_l_elbow_y_name = "left_elbow_joint";
  params.j_r_elbow_y_name = "right_elbow_joint";
  params.contactNames6DoF = {"left_test_contact", "right_test_contact"};
  params.contactParentJointNames = {"left_ankle_pitch_joint", "right_ankle_pitch_joint"};
  const std::string urdf = "src/legged_robot_mpc_controller/description/urdf/robots/h2/h2_31dof.urdf";
  ModelSettings settings(params, urdf, "native_tangent_");
  WbIdMpcRobotModel<double> robot(settings, true);
  auto pin = createCustomPinocchioInterface(urdf, settings, false, 1.0, false, true);
  const auto& model = pin.getModel();
  const auto nv = robot.getGenCoordinatesDim();
  const auto nx = robot.getStateDim();
  check(model.nq == model.nv + 1, "Pinocchio quaternion storage dimension");
  // The two coordinate choices must describe identical physical inverse dynamics.
  {
    auto eulerPin = createCustomPinocchioInterface(urdf, settings, false, 1.0, false, false);
    for (const vector3_t angles : {vector3_t(0.7, 0.1, -0.2), vector3_t(-2.2, 1.2, 0.4)}) {
      vector_t physicalState = vector_t::Zero(nx);
      physicalState(nx - 1) = 1.0;
      robot.setBaseOrientationEulerZYX(physicalState, angles);
      const vector_t sphericalQ = robot.getGeneralizedCoordinates(physicalState);
      vector_t eulerQ = vector_t::Zero(nv);
      eulerQ.segment<3>(3) = angles;
      vector_t eulerV = vector_t::Constant(nv, 0.1);
      vector_t eulerA = vector_t::Constant(nv, -0.2);
      eulerV.segment<3>(3) << 0.3, -0.2, 0.4;
      eulerA.segment<3>(3) << -0.1, 0.4, 0.2;
      vector_t sphericalV = eulerV, sphericalA = eulerA;
      const matrix3_t rotation = robot.getBaseQuaternion(physicalState).toRotationMatrix();
      sphericalV.segment<3>(3) = getLocalAngularVelocityFromEulerAnglesZyxDerivatives<double>(
          angles, vector3_t(eulerV.segment<3>(3)));
      sphericalA.segment<3>(3) = rotation.transpose() * getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<double>(
          angles, vector3_t(eulerV.segment<3>(3)), vector3_t(eulerA.segment<3>(3)));
      const vector_t sphericalTau = pinocchio::rnea(model, pin.getData(), sphericalQ, sphericalV, sphericalA);
      const vector_t eulerTau = pinocchio::rnea(eulerPin.getModel(), eulerPin.getData(), eulerQ, eulerV, eulerA);
      vector_t transformed = sphericalTau;
      transformed.segment<3>(3) = getMappingFromEulerAnglesZyxDerivativeToLocalAngularVelocity<double>(angles).transpose() *
                                  sphericalTau.segment<3>(3);
      check((transformed - eulerTau).norm() < 1e-8, "Euler and tangent RNEA agree after physical coordinate conversion");
    }
  }
  WbIdTangentDynamics dynamics(robot);
  WbIdPinocchioMapping mapping(robot);
  vector_t input = vector_t::Zero(robot.getInputDim());
  vector_t state = vector_t::Zero(nx);
  state(nx - 1) = 1.0;
  state(2) = 1.02;
  state.segment<3>(nv) << 0.2, -0.1, 0.03;
  state.segment<3>(nv + 3) << 0.3, -0.2, 0.4;
  PreComputation pre;
  const auto discretize = selectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);
  for (const double scale : {0.0, 1e-20, 1e-15, 1e-10, 1e-5, 1.0}) {
    state.segment<3>(3).setZero();
    state.segment<3>(nv + 3) = scale * vector3_t(0.3, -0.2, 0.4);
    const auto flow = dynamics.linearApproximation(0, state, input, pre);
    const auto step = discretize(dynamics, 0, state, input, 0.02);
    std::cout << "Zero increment, omega scale=" << scale << " flowJ=" << flow.dfdx.allFinite()
              << " RK4J=" << step.dfdx.allFinite() << std::endl;
    check(flow.dfdx.allFinite() && flow.dfdu.allFinite(), "finite flow at zero increment and varying angular velocity");
    check(step.f.allFinite() && step.dfdx.allFinite() && step.dfdu.allFinite(), "finite RK4 sensitivity at zero increment");
  }
  state.segment<3>(nv + 3) << 0.3, -0.2, 0.4;
  for (const vector3_t& angles : {vector3_t(0.7, 0.0, 0.0), vector3_t(1.3, 1.5707963267948966, -0.3),
                                 vector3_t(-2.5, -1.6, 0.4)}) {
    robot.setBaseOrientationEulerZYX(state, angles);
    state.segment<3>(3) << 0.2, -0.15, 0.3;
    const vector_t q = robot.getGeneralizedCoordinates(state);
    const vector_t velocity = robot.getGeneralizedVelocities(state);
    const auto orientation = robot.getBaseQuaternion(state);
    check(std::abs(orientation.norm() - 1) < 1e-12, "unit orientation");
    pinocchio::forwardKinematics(model, pin.getData(), q, velocity);
    const auto base = model.getFrameId("pelvis");
    const auto pose = pinocchio::updateFramePlacement(model, pin.getData(), base);
    const auto twist = pinocchio::getFrameVelocity(model, pin.getData(), base, pinocchio::LOCAL_WORLD_ALIGNED);
    check((pose.rotation() - orientation.toRotationMatrix()).norm() < 1e-10, "base rotation mapping");
    check((twist.linear() - velocity.head<3>()).norm() < 1e-10, "world linear velocity mapping");
    check((twist.angular() - orientation * velocity.segment<3>(3)).norm() < 1e-10, "body angular velocity mapping");
    const auto linear = dynamics.linearApproximation(0, state, input, pre);
    check(linear.f.allFinite() && linear.dfdx.allFinite() && linear.dfdu.allFinite(), "finite auto-generated flow Jacobian");
    check(linear.f.tail<4>().norm() == 0, "anchor has zero dynamics");
    const double dt = 1e-7;
    const vector_t qNext = pinocchio::integrate(model, q, vector_t(dt * velocity));
    const vector_t advanced = state + dt * linear.f;
    const vector_t actualDelta = pinocchio::difference(model, qNext, robot.getGeneralizedCoordinates(advanced));
    check(actualDelta.norm() < 1e-12, "chart kinematics agrees with Pinocchio integration");
    const matrix_t identity = matrix_t::Identity(nv, nv);
    const matrix_t zeros = matrix_t::Zero(nv, nv);
    const auto mapped = mapping.getOcs2Jacobian(state, identity, zeros).first;
    const double eps = 1e-6;
    for (Eigen::Index col = 0; col < state.size(); ++col) {
      vector_t plus = state, minus = state;
      plus(col) += eps;
      minus(col) -= eps;
      const vector_t actual = pinocchio::difference(model, robot.getGeneralizedCoordinates(minus),
                                                   robot.getGeneralizedCoordinates(plus)) / (2 * eps);
      check((actual - mapped.col(col)).norm() < 1e-7, "Pinocchio state-map Jacobian");
    }
    const vector_t original = robot.getGeneralizedCoordinates(state);
    robot.rebase(state, Eigen::Quaterniond(Eigen::AngleAxisd(-2.9, vector3_t::UnitZ())));
    check(pinocchio::difference(model, original, robot.getGeneralizedCoordinates(state)).norm() < 1e-10,
          "rebasing preserves physical configuration");
  }
  auto schedule = GaitSchedule::createGaitSchedule(ModeSchedule{{0.5}, {STANCE, STANCE}},
      ModeSequenceTemplate{{0.0, 0.5}, {STANCE}}, settings);
  auto swing = std::make_unique<SwingTrajectoryPlanner>(SwingTrajectoryPlanner::Config{}, N_CONTACTS);
  SwitchedModelReferenceManager reference(schedule, std::move(swing), pin, robot);
  WbIdMpcRobotModel<ad_scalar_t> adRobot(settings, true);
  WbIdBaseMotionCost baseCost(matrix_t::Identity(12, 12), reference, robot, adRobot);
  state.setZero();
  state(nx - 1) = 1.0;
  state(2) = 1.02;
  TargetTrajectories targets({0.0}, {state}, {input});
  const auto zero = baseCost.getQuadraticApproximation(0, state, targets, pre);
  if (!(std::abs(zero.f) < 1e-12 && zero.dfdx.allFinite() && zero.dfdxx.allFinite())) {
    std::cerr << "Identity base cost f=" << zero.f << "\ngradient=" << zero.dfdx.transpose() << '\n';
    WbIdBaseMotionResidual residual(matrix_t::Identity(12, 12), reference, robot, adRobot);
    const auto jacobian = residual.getQuadraticApproximation(0, state, input, targets, pre);
    std::cerr << "input gradient=" << jacobian.dfdu.transpose() << '\n';
    CppAdInterface inspection([](const ad_vector_t&, const ad_vector_t&, ad_vector_t&) {},
        1 + nx + robot.getInputDim(), nx + 13, "tangent_base_motion_v3", settings.modelFolderCppAd);
    inspection.loadModels(false);
    vector_t packed(1 + nx + robot.getInputDim());
    packed << 0.0, state, input;
    const auto raw = inspection.getJacobian(packed, residual.getParameters(0, targets, pre));
    std::cerr << "Raw residual increment Jacobian:\n" << raw.middleCols(4, 3) << '\n';
  }
  check(std::abs(zero.f) < 1e-12 && zero.dfdx.allFinite() && zero.dfdxx.allFinite(), "finite generated base cost at identity");
  state.segment<3>(3) << 0.1, -0.15, 0.3;
  state.segment<3>(nv) << 0.3, -0.2, 0.1;
  state.segment<3>(nv + 3) << 0.1, 0.3, -0.2;
  const auto baseline = baseCost.getQuadraticApproximation(0, state, targets, pre);
  check(baseline.f > 0 && baseline.dfdx.allFinite() && baseline.dfdxx.allFinite(), "finite generated nonzero base cost");
  vector_t negativeAnchor = state;
  negativeAnchor.tail<4>() *= -1;
  check(std::abs(baseCost.getValue(0, negativeAnchor, targets, pre) - baseline.f) < 1e-10,
        "base cost is invariant to quaternion storage sign");
  const double eps = 1e-6;
  for (Eigen::Index col = 0; col < state.size(); ++col) {
    vector_t plus = state, minus = state;
    plus(col) += eps;
    minus(col) -= eps;
    const double actual = (baseCost.getValue(0, plus, targets, pre) - baseCost.getValue(0, minus, targets, pre)) / (2 * eps);
    check(std::abs(actual - baseline.dfdx(col)) < 1e-7, "generated base-cost gradient");
  }
  const Eigen::Quaterniond gauge(Eigen::AngleAxisd(2.4, vector3_t::UnitZ()));
  const auto changeGauge = [&](vector_t& x) {
    x.head<3>() = gauge * vector3_t(x.head<3>()) + vector3_t(3, -2, 0);
    x.tail<4>() = (gauge * Eigen::Quaterniond(x.tail<4>())).coeffs();
    x.segment<3>(nv) = gauge * vector3_t(x.segment<3>(nv));
  };
  changeGauge(state);
  changeGauge(targets.stateTrajectory[0]);
  check(std::abs(baseCost.getValue(0, state, targets, pre) - baseline.f) < 1e-10, "base cost preserves planar gauge");
  CppAdInterface absoluteCost([](const ad_vector_t&, const ad_vector_t&, ad_vector_t&) {},
      1 + nx + robot.getInputDim(), nx + 13, "tangent_base_motion_v3", settings.modelFolderCppAd);
  absoluteCost.loadModels(false);
  for (const double pitch : {1.5707963267948966, -1.5707963267948966, 1.8}) {
    robot.setBaseOrientationEulerZYX(state, vector3_t(0.0, pitch, 0.0));
    vector_t parameters(nx + 13), packed(1 + nx + robot.getInputDim());
    parameters << state, vector_t::Ones(12), 0.0;
    packed << 0.0, state, input;
    check(absoluteCost.getFunctionValue(packed, parameters).allFinite() &&
          absoluteCost.getJacobian(packed, parameters).allFinite(), "finite full-pose cost at/past vertical pitch");
  }
  std::cout << "Tangent model: FK/twist/integration, generated flow/base-cost gradients, mapping, rebasing, sign and gauge checks passed.\n";
}
