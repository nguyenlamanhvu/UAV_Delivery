#include "systems/moving_target_lcm_systems.h"

#include <cmath>

#include "drake/common/value.h"

namespace uav_delivery {
namespace systems {
namespace {

void FillTimestamps(const drake::systems::Context<double>& context,
                    int64_t* utime,
                    int64_t* timestamp_millis) {
  *utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  *timestamp_millis = static_cast<int64_t>(std::llround(context.get_time() * 1e3));
}

}  // namespace

MovingTargetTeleopReceiver::MovingTargetTeleopReceiver() {
  input_port_ = this->DeclareAbstractInputPort(
      "lcmt_moving_target_teleop_command",
      drake::Value<lcmt_moving_target_teleop_command>{})
                    .get_index();
  this->DeclareVectorOutputPort("teleop", drake::systems::BasicVector<double>(3),
                                &MovingTargetTeleopReceiver::CopyTeleopOut);
}

void MovingTargetTeleopReceiver::CopyTeleopOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& msg = this->get_input_port(input_port_)
                        .Eval<lcmt_moving_target_teleop_command>(context);
  Eigen::Vector3d cmd(msg.throttle, msg.turn, msg.stop ? 1.0 : 0.0);
  output->SetFromVector(cmd);
}

MovingTargetActuationReceiver::MovingTargetActuationReceiver() {
  input_port_ = this->DeclareAbstractInputPort(
      "lcmt_moving_target_actuation_command",
      drake::Value<lcmt_moving_target_actuation_command>{})
                    .get_index();
  this->DeclareVectorOutputPort(
      "drive_torque", drake::systems::BasicVector<double>(2),
      &MovingTargetActuationReceiver::CopyActuationOut);
}

void MovingTargetActuationReceiver::CopyActuationOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& msg = this->get_input_port(input_port_)
                        .Eval<lcmt_moving_target_actuation_command>(context);
  Eigen::Vector2d command(msg.left_drive_torque, msg.right_drive_torque);
  output->SetFromVector(command);
}

MovingTargetStateReceiver::MovingTargetStateReceiver() {
  input_port_ = this->DeclareAbstractInputPort(
      "lcmt_moving_target_state", drake::Value<lcmt_moving_target_state>{})
                    .get_index();
  this->DeclareVectorOutputPort("state", drake::systems::BasicVector<double>(9),
                                &MovingTargetStateReceiver::CopyStateOut);
}

void MovingTargetStateReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& msg =
      this->get_input_port(input_port_).Eval<lcmt_moving_target_state>(context);
  Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
  state(0) = msg.x;
  state(1) = msg.y;
  state(2) = msg.yaw;
  state(3) = std::cos(msg.yaw) * msg.vx + std::sin(msg.yaw) * msg.vy;
  state(4) = msg.yaw_rate;
  state(5) = msg.left_wheel_angle_front;
  state(6) = msg.right_wheel_angle_front;
  state(7) = msg.left_wheel_angle_rear;
  state(8) = msg.right_wheel_angle_rear;
  output->SetFromVector(state);
}

MovingTargetActuationSender::MovingTargetActuationSender() {
  input_port_ = this->DeclareVectorInputPort(
      "drive_torque", drake::systems::BasicVector<double>(2))
                    .get_index();
  this->DeclareAbstractOutputPort("lcmt_moving_target_actuation_command",
                                  &MovingTargetActuationSender::CalcMessage);
}

void MovingTargetActuationSender::CalcMessage(
    const drake::systems::Context<double>& context,
    lcmt_moving_target_actuation_command* output) const {
  const Eigen::VectorXd value = this->get_input_port(input_port_).Eval(context);
  FillTimestamps(context, &output->utime, &output->timestamp_millis);
  output->left_drive_torque = value(0);
  output->right_drive_torque = value(1);
}

MovingTargetStateSender::MovingTargetStateSender() {
  input_port_ = this->DeclareVectorInputPort(
      "state", drake::systems::BasicVector<double>(9))
                    .get_index();
  this->DeclareAbstractOutputPort("lcmt_moving_target_state",
                                  &MovingTargetStateSender::CalcMessage);
}

void MovingTargetStateSender::CalcMessage(const drake::systems::Context<double>& context,
                                          lcmt_moving_target_state* output) const {
  const Eigen::VectorXd value = this->get_input_port(input_port_).Eval(context);
  FillTimestamps(context, &output->utime, &output->timestamp_millis);
  output->x = value(0);
  output->y = value(1);
  output->yaw = value(2);
  output->vx = value(3) * std::cos(value(2));
  output->vy = value(3) * std::sin(value(2));
  output->yaw_rate = value(4);
  output->left_wheel_angle_front = value(5);
  output->left_wheel_angle_rear = value(7);
  output->right_wheel_angle_front = value(6);
  output->right_wheel_angle_rear = value(8);
}

std::vector<std::string> MovingTargetStateNames() {
  return {"x",         "y",         "yaw",      "forward_speed", "yaw_rate",
          "wheel_fl",  "wheel_fr",  "wheel_rl", "wheel_rr"};
}

}  // namespace systems
}  // namespace uav_delivery
