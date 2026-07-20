#include "systems/moving_target_plant.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace uav_delivery {
namespace systems {

MovingTargetPlant::MovingTargetPlant(MovingTargetPlantParams params)
    : params_(std::move(params)) {
  command_port_ = this->DeclareVectorInputPort(
      "drive_torque", drake::systems::BasicVector<double>(2))
                      .get_index();
  this->DeclareContinuousState(kStateSize);
  this->DeclareVectorOutputPort("state",
                                drake::systems::BasicVector<double>(kStateSize),
                                &MovingTargetPlant::CopyState);
}

const drake::systems::InputPort<double>& MovingTargetPlant::get_command_input_port()
    const {
  return this->get_input_port(command_port_);
}

void MovingTargetPlant::DoCalcTimeDerivatives(
    const drake::systems::Context<double>& context,
    drake::systems::ContinuousState<double>* derivatives) const {
  const Eigen::VectorXd x = context.get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd u = this->get_command_input_port().Eval(context);
  const double tau_l = std::clamp(u(0), -params_.max_drive_torque,
                                  params_.max_drive_torque);
  const double tau_r = std::clamp(u(1), -params_.max_drive_torque,
                                  params_.max_drive_torque);

  const double yaw = x(2);
  const double v = x(3);
  const double yaw_rate = x(4);

  const double v_dot = params_.k_drive * 0.5 * (tau_l + tau_r) - params_.d_v * v;
  const double yaw_rate_dot =
      params_.k_turn * (tau_r - tau_l) - params_.d_yaw * yaw_rate;
  const double wheel_angle_dot_left =
      v / std::max(params_.wheel_radius, 1e-6) + params_.k_wheel * tau_l;
  const double wheel_angle_dot_right =
      v / std::max(params_.wheel_radius, 1e-6) + params_.k_wheel * tau_r;

  Eigen::VectorXd xdot = Eigen::VectorXd::Zero(kStateSize);
  xdot(0) = v * std::cos(yaw);
  xdot(1) = v * std::sin(yaw);
  xdot(2) = yaw_rate;
  xdot(3) = v_dot;
  xdot(4) = yaw_rate_dot;
  xdot(5) = wheel_angle_dot_left;
  xdot(6) = wheel_angle_dot_right;
  xdot(7) = wheel_angle_dot_left;
  xdot(8) = wheel_angle_dot_right;
  derivatives->SetFromVector(xdot);
}

void MovingTargetPlant::CopyState(const drake::systems::Context<double>& context,
                                  drake::systems::BasicVector<double>* output) const {
  output->SetFromVector(context.get_continuous_state_vector().CopyToVector());
}

}  // namespace systems
}  // namespace uav_delivery
