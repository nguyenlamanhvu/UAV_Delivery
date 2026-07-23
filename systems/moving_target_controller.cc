#include "systems/moving_target_controller.h"

#include <algorithm>
#include <utility>

namespace uav_delivery {
namespace systems {

MovingTargetController::MovingTargetController(MovingTargetControllerParams params)
    : params_(std::move(params)) {
  teleop_port_ = this->DeclareVectorInputPort(
      "teleop", drake::systems::BasicVector<double>(3))
                     .get_index();
  state_port_ = this->DeclareVectorInputPort(
      "state", drake::systems::BasicVector<double>(9))
                    .get_index();
  this->DeclareVectorOutputPort("drive_torque",
                                drake::systems::BasicVector<double>(2),
                                &MovingTargetController::CalcDriveTorques);
}

void MovingTargetController::CalcDriveTorques(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const Eigen::VectorXd cmd = this->get_input_port(teleop_port_).Eval(context);
  const Eigen::VectorXd state = this->get_input_port(state_port_).Eval(context);
  (void)state;
  const double throttle = std::clamp(cmd(0), -1.0, 1.0);
  const double turn = std::clamp(cmd(1), -1.0, 1.0);
  const bool stop = cmd(2) > 0.5;

  double left = 0.0;
  double right = 0.0;
  if (!stop) {
    left = params_.throttle_gain * throttle - params_.turn_gain * turn;
    right = params_.throttle_gain * throttle + params_.turn_gain * turn;
    left = std::clamp(left, -params_.max_drive_torque, params_.max_drive_torque);
    right = std::clamp(right, -params_.max_drive_torque, params_.max_drive_torque);
  }

  Eigen::Vector2d torques(left, right);
  output->SetFromVector(torques);
}

}  // namespace systems
}  // namespace uav_delivery
