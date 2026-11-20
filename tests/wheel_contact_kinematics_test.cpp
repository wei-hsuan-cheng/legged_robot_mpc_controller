#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "legged_robot_mpc_controller/mpc/wb_id/model/WheelContactKinematics.h"

using Eigen::Vector3d;
namespace wheel = ocs2::humanoid::wheel;

int main() {
  try {
    const Vector3d normal = Vector3d::UnitZ();
    const Vector3d axle = Vector3d(0.2, 0.95, 0.15).normalized();
    const Vector3d omega(0.4, -0.3, 1.2);
    const Vector3d axleRate = omega.cross(axle);
    const Vector3d lateral = wheel::lateralDirection<double>(axle, normal);
    const Vector3d lateralRate = wheel::lateralDirectionRate<double>(axle, axleRate, normal);
    const Vector3d rhoRate = wheel::contactOffsetRate<double>(axle, axleRate, normal, .055);
    const double dt = 1e-6;
    const auto rotate = [&](double time) {
      return (Eigen::AngleAxisd(time * omega.norm(), omega.normalized()) * axle).eval();
    };
    const Vector3d plus = rotate(dt), minus = rotate(-dt);
    const Vector3d lateralDifference = (wheel::lateralDirection<double>(plus, normal) -
                                      wheel::lateralDirection<double>(minus, normal)) / (2 * dt);
    const Vector3d rhoDifference = (wheel::contactOffset<double>(plus, normal, .055) -
                                  wheel::contactOffset<double>(minus, normal, .055)) / (2 * dt);
    if ((lateralRate - lateralDifference).norm() > 1e-9 || (rhoRate - rhoDifference).norm() > 1e-9) {
      throw std::runtime_error("wheel geometry derivatives disagree with finite differences");
    }
    const Vector3d slip(.3, -.2, .1), slipRate(-.4, .8, -.1);
    const double residualRate = lateral.dot(slipRate) + lateralRate.dot(slip);
    const double difference = (wheel::lateralDirection<double>(plus, normal).dot(slip + dt * slipRate) -
                               wheel::lateralDirection<double>(minus, normal).dot(slip - dt * slipRate)) / (2 * dt);
    if (std::abs(residualRate - difference) > 1e-9) {
      throw std::runtime_error("rotating-direction slip derivative is incorrect");
    }
    const Vector3d opposite = -axle;
    const double oldResidual = .5 * (lateral.dot(slip) + wheel::lateralDirection<double>(opposite, normal).dot(slip));
    const double commonResidual = .5 * (lateral.dot(slip) + lateral.dot(slip));
    if (std::abs(oldResidual) > 1e-12 || std::abs(commonResidual) < .01) {
      throw std::runtime_error("mirrored-axis cancellation regression check failed");
    }
    const Vector3d spinRate = (12.0 * axle).cross(axle);
    if (wheel::lateralDirectionRate<double>(axle, spinRate, normal).norm() > 1e-12) {
      throw std::runtime_error("pure spin must not rotate the lateral basis");
    }
    std::cout << "Wheel offset/basis derivatives and mirrored-axis regression passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
