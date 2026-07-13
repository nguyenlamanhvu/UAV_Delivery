#include "systems/hover_controller.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace uav_delivery {
namespace systems {

HoverController::HoverController(QuadrotorSimParams params)
    : params_(std::move(params)) {
  state_port_ = this->DeclareVectorInputPort(
      "quadrotor_state", drake::systems::BasicVector<double>(12))
                    .get_index();
  this->DeclareVectorOutputPort("rotor_speed_command_rad_per_sec",
                                drake::systems::BasicVector<double>(4),
                                &HoverController::CalcCommand);
}

void HoverController::CalcCommand(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const Eigen::VectorXd state = this->get_input_port(state_port_).Eval(context);
  const Eigen::Vector3d rpy = state.segment<3>(3);
  const Eigen::Vector3d omega_B = state.segment<3>(9);
  const double z = state(2);
  const double vz = state(8);

  const Eigen::Vector3d moment_cmd =
      -params_.hover_controller.kp_rpy.cwiseProduct(rpy) -
      params_.hover_controller.kd_angular.cwiseProduct(omega_B);

  const double altitude_accel =
      params_.hover_controller.kp_z * (params_.hover_controller.desired_z - z) -
      params_.hover_controller.kd_z * vz;
  const double thrust =
      params_.plant.mass * (params_.plant.gravity + altitude_accel);

  output->SetFromVector(MixToRotorSpeeds(thrust, moment_cmd));
}

Eigen::Vector4d HoverController::MixToRotorSpeeds(
    double thrust, const Eigen::Vector3d& moment_B) const {
  const double l = params_.plant.arm_length;
  const double b = params_.plant.thrust_coeff;
  const double d = params_.plant.yaw_moment_coeff;

  Eigen::Matrix4d mixer;
  mixer << b, b, b, b,
           0.0, l * b, 0.0, -l * b,
           -l * b, 0.0, l * b, 0.0,
           d, -d, d, -d;

  Eigen::Vector4d wrench;
  wrench << thrust, moment_B.x(), moment_B.y(), moment_B.z();
  Eigen::Vector4d omega_squared = mixer.fullPivLu().solve(wrench);

  Eigen::Vector4d rotor_speed = Eigen::Vector4d::Zero();
  const double max_squared =
      params_.plant.max_rotor_speed * params_.plant.max_rotor_speed;
  for (int i = 0; i < 4; ++i) {
    omega_squared(i) = std::clamp(omega_squared(i), 0.0, max_squared);
    rotor_speed(i) = std::sqrt(omega_squared(i));
  }
  return rotor_speed;
}

}  // namespace systems
}  // namespace uav_delivery
