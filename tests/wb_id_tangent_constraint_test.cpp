#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_wb_id_mpc/constraint/WbIdRneaConstraintCppAd.h>
#include <humanoid_wb_id_mpc/constraint/WbIdFootAccelerationConstraintCppAd.h>
#include <iostream>
#include <stdexcept>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  ModelSettings::Params params;
  params.robotName = "h2";
  params.verboseCppAd = false;
  params.recompileLibrariesCppAd = true;
  params.modelFolderCppAd = "/tmp/wb_id_tangent_constraints";
  params.j_l_shoulder_y_name = "left_shoulder_pitch_joint";
  params.j_r_shoulder_y_name = "right_shoulder_pitch_joint";
  params.j_l_elbow_y_name = "left_elbow_joint";
  params.j_r_elbow_y_name = "right_elbow_joint";
  params.contactNames6DoF = {"left_test_contact", "right_test_contact"};
  params.contactParentJointNames = {"left_ankle_pitch_joint", "right_ankle_pitch_joint"};
  const std::string urdf = "src/legged_robot_mpc_controller/description/urdf/robots/h2/h2_31dof.urdf";
  ModelSettings settings(params, urdf, "native_constraint_");
  WbIdMpcRobotModel<double> robot(settings, true);
  WbIdMpcRobotModel<ad_scalar_t> adRobot(settings, true);
  auto pin = createCustomPinocchioInterface(urdf, settings, false, 1.0, false, true);
  auto window = std::make_shared<WbIdTorqueWindow>(-1.0);
  WbIdRneaConstraintCppAd base(WbIdRneaConstraintCppAd::Rows::Base, pin, adRobot, settings);
  WbIdRneaConstraintCppAd joints(WbIdRneaConstraintCppAd::Rows::Joint, pin, adRobot, settings, window);
  auto schedule = GaitSchedule::createGaitSchedule(ModeSchedule{{0.5}, {STANCE, STANCE}},
      ModeSequenceTemplate{{0.0, 0.5}, {STANCE}}, settings);
  auto swingPlanner = std::make_unique<SwingTrajectoryPlanner>(SwingTrajectoryPlanner::Config{}, N_CONTACTS);
  SwitchedModelReferenceManager reference(schedule, std::move(swingPlanner), pin, robot);
  WbIdFootAccelerationConstraintCppAd::Gains gains;
  gains.positionZ = 50;
  gains.orientation = 100;
  gains.linearVelocityXY = gains.linearVelocityZ = gains.angularVelocity = 50;
  WbIdFootAccelerationConstraintCppAd stance(WbIdFootAccelerationConstraintCppAd::Type::Stance, reference, pin,
                                           adRobot, 0, gains, settings);
  WbIdFootAccelerationConstraintCppAd swing(WbIdFootAccelerationConstraintCppAd::Type::SwingVertical, reference, pin,
                                          adRobot, 1, gains, settings);
  const std::vector<StateInputConstraintCppAd*> constraints{&base, &joints, &stance, &swing};
  const auto nx = robot.getStateDim(), nu = robot.getInputDim(), nv = robot.getGenCoordinatesDim();
  PreComputation pre;
  for (double pitch : {0.0, 1.2}) {
    for (double increment : {0.0, 1e-8, 0.2}) {
      vector_t state = vector_t::Zero(nx), input = vector_t::Zero(nu);
      robot.setBaseOrientationEulerZYX(state, vector3_t(0.7, pitch, -0.2));
      state.segment<3>(3) << increment, -increment, 0.5 * increment;
      state.segment(6, nv - 6) = vector_t::LinSpaced(nv - 6, -0.2, 0.4);
      state.segment(nv, nv) = vector_t::LinSpaced(nv, -0.4, 0.7);
      input.head<12>() << 100, -50, 400, 3, 1, 4, 20, 40, 300, 2, -1, -3;
      input.segment(12, nv) = vector_t::LinSpaced(nv, -1.0, 2.0);
      input.tail(nv - 6) = vector_t::LinSpaced(nv - 6, -20, 25);
      reference.setTargetTrajectories(TargetTrajectories({0.0}, {state}, {input}));
      reference.preSolverRun(0.0, 1.0, state);
      const vector_t q = robot.getGeneralizedCoordinates(state);
      const auto& model = pin.getModel();
      auto& data = pin.getData();
      vector_t expected = pinocchio::rnea(model, data, q, robot.getGeneralizedVelocities(state),
                                         robot.getGeneralizedAccelerations(input));
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      for (size_t foot = 0; foot < 2; ++foot) {
        matrix_t jacobian = matrix_t::Zero(6, nv);
        pinocchio::getFrameJacobian(model, data, model.getFrameId(settings.contactNames[foot]),
                                    pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
        expected -= jacobian.transpose() * robot.getContactWrench(input, foot);
      }
      expected.tail(nv - 6) -= robot.getJointTorques(input);
      double worst = 0.0;
      for (auto* constraint : constraints) {
        const auto linear = constraint->getLinearApproximation(0, state, input, pre);
        const vector_t value = constraint == &base ? vector_t(expected.head<6>()) : vector_t(expected.tail(nv - 6));
        if (!linear.f.allFinite() || !linear.dfdx.allFinite() || !linear.dfdu.allFinite() ||
            ((constraint == &base || constraint == &joints) && (linear.f - value).norm() > 1e-8))
          throw std::runtime_error("generated RNEA value/Jacobian is inconsistent with native physics");
        for (size_t col = 0; col < nx + nu; ++col) {
          vector_t xp = state, xm = state, up = input, um = input;
          const double eps = 1e-6;
          if (col < nx) { xp(col) += eps; xm(col) -= eps; }
          else { up(col - nx) += eps; um(col - nx) -= eps; }
          const vector_t actual = (constraint->getValue(0, xp, up, pre) - constraint->getValue(0, xm, um, pre)) / (2 * eps);
          const vector_t predicted = col < nx ? vector_t(linear.dfdx.col(col)) : vector_t(linear.dfdu.col(col - nx));
          if (!actual.allFinite() || !predicted.allFinite())
            throw std::runtime_error("nonfinite constraint perturbation or generated derivative");
          const double error = ((actual - predicted).array().abs() / (1.0 + actual.array().abs())).maxCoeff();
          worst = std::max(worst, error);
          if (error > 1e-5) {
            std::cerr << "pitch=" << pitch << " phi=" << increment << " col=" << col << " relative error=" << error << '\n';
            throw std::runtime_error("generated RNEA Jacobian failed independent perturbation check");
          }
        }
      }
      std::cout << "RNEA pitch=" << pitch << " phi=" << increment << " worst scaled derivative error=" << worst << '\n';
    }
  }
}
