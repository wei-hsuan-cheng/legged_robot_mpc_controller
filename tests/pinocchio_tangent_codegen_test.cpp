// Include the AD backend before Pinocchio algorithms so its specializations are visible.
#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/So3Tangent.h>
#include <Eigen/SVD>
#include <iostream>

int main() {
  using namespace ocs2;
  using V3 = Eigen::Vector3d;
  using AV3 = Eigen::Matrix<ad_scalar_t, 3, 1>;
  CppAdInterface model([](const ad_vector_t& x, ad_vector_t& y) {
    const AV3 phi = x.head<3>();
    const auto q = pinocchio::quaternion::exp3(phi);
    const AV3 logarithm = pinocchio::quaternion::log3(q);
    const Eigen::Matrix<ad_scalar_t, 3, 3> inverse = so3RightJacobianInverse(phi);
    y.resize(12);
    y.head<3>() = logarithm;
    Eigen::Map<Eigen::Matrix<ad_scalar_t, 9, 1>>(y.data() + 3) = Eigen::Map<const Eigen::Matrix<ad_scalar_t, 9, 1>>(inverse.data());
  }, 3, "pinocchio_tangent_probe", "/tmp/pinocchio_tangent_codegen_test");
  model.createModels(CppAdInterface::ApproximationOrder::First, false);
  CppAdInterface rawQuaternion([](const ad_vector_t& x, ad_vector_t& y) {
    const Eigen::Matrix<ad_scalar_t, 4, 1> coefficients = x;
    y = pinocchio::quaternion::log3(Eigen::Quaternion<ad_scalar_t>(coefficients));
  }, 4, "pinocchio_raw_quaternion_log_probe", "/tmp/pinocchio_tangent_codegen_test");
  rawQuaternion.createModels(CppAdInterface::ApproximationOrder::First, false);
  const vector_t identity = (vector_t(4) << 0.0, 0.0, 0.0, 1.0).finished();
  std::cout << "Raw quaternion log Jacobian at identity:\n" << rawQuaternion.getJacobian(identity) << '\n';
  if (!rawQuaternion.getJacobian(identity).allFinite()) return 3;
  CppAdInterface angle([](const ad_vector_t& x, ad_vector_t& y) {
    y.resize(1);
    y(0) = atan2(x(0), x(1));
  }, 2, "atan2_axis_probe", "/tmp/pinocchio_tangent_codegen_test");
  angle.createModels(CppAdInterface::ApproximationOrder::Second, false);
  for (const Eigen::Vector2d& xy : {Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 0.0),
                                  Eigen::Vector2d(-1.0, 0.0), Eigen::Vector2d(0.0, -1.0),
                                  Eigen::Vector2d(0.2, 0.8), Eigen::Vector2d(-0.2, 0.8),
                                  Eigen::Vector2d(0.2, -0.8), Eigen::Vector2d(-0.2, -0.8),
                                  Eigen::Vector2d(0.5, 0.5), Eigen::Vector2d(-0.5, -0.5)}) {
    const auto jacobian = angle.getJacobian(xy);
    const Eigen::RowVector2d expected(xy(1) / xy.squaredNorm(), -xy(0) / xy.squaredNorm());
    std::cout << "atan2(y,x)=" << xy.transpose() << " J=" << jacobian << '\n';
    if (!jacobian.allFinite() || (jacobian - expected).norm() > 1e-8) return 4;
    const double y = xy(0), x = xy(1), r4 = xy.squaredNorm() * xy.squaredNorm();
    Eigen::Matrix2d expectedHessian;
    expectedHessian << -2 * x * y / r4, (y * y - x * x) / r4,
                       (y * y - x * x) / r4, 2 * x * y / r4;
    const auto hessian = angle.getHessian(size_t(0), xy);
    if (!hessian.allFinite() || (hessian - expectedHessian).norm() > 1e-8) return 5;
  }
  for (const V3& phi : {V3::Zero().eval(), V3(1e-8, -2e-8, 3e-8), V3(0.2, 1.5707963267948966, -0.3), V3(2.8, 0.2, -0.1)}) {
    const auto value = model.getFunctionValue(phi);
    const auto jacobian = model.getJacobian(phi);
    const auto expected = pinocchio::log3(pinocchio::exp3(phi));
    if (!value.allFinite() || !jacobian.allFinite() || (value.head<3>() - expected).norm() > 1e-9 ||
        (jacobian.topRows(3) - Eigen::Matrix3d::Identity()).norm() > 1e-7) {
      std::cerr << "FAILED phi=" << phi.transpose() << " value=" << value.transpose() << "\nJ=\n" << jacobian << '\n';
      return 1;
    }
    Eigen::Matrix3d expectedInverse;
    pinocchio::Jlog3(pinocchio::exp3(phi), expectedInverse);
    if ((Eigen::Map<const Eigen::Matrix3d>(value.data() + 3) - expectedInverse).norm() > 1e-8) return 2;
  }
  for (const double degrees : {0.0, 60.0, 80.0, 89.0, 89.9}) {
    Eigen::Matrix3d rateMap;
    const V3 angles(0.3, degrees * std::acos(-1.0) / 180.0, -0.1);
    for (int col = 0; col < 3; ++col) {
      rateMap.col(col) = getEulerAnglesZyxDerivativesFromLocalAngularVelocity<double>(angles, V3::Unit(col));
    }
    const auto singular = rateMap.jacobiSvd().singularValues();
    std::cout << "Absolute pitch=" << degrees << " deg: Euler rate-map condition=" << singular(0) / singular(2)
              << "; re-anchored tangent rate-map condition=1\n";
  }
  std::cout << "Pinocchio exp/log, GTSAM-form inverse dexp and atan2 generated derivative tests passed.\n";
}
