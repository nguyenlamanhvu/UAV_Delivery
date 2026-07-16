#include "systems/lcm_systems.h"

#include <cmath>
#include <utility>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/value.h"
#include "params/quadrotor_params.h"

namespace uav_delivery {
namespace systems {

QuadrotorCommandReceiver::QuadrotorCommandReceiver() {
  input_port_ = this->DeclareAbstractInputPort(
      "lcmt_quadrotor_command", drake::Value<lcmt_quadrotor_command>{})
                    .get_index();
  this->DeclareVectorOutputPort("rotor_input",
                                drake::systems::BasicVector<double>(4),
                                &QuadrotorCommandReceiver::CopyCommandOut);
}

void QuadrotorCommandReceiver::CopyCommandOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& msg =
      this->get_input_port(input_port_).Eval<lcmt_quadrotor_command>(context);
  Eigen::Vector4d command = Eigen::Vector4d::Zero();
  for (int i = 0; i < 4; ++i) {
    command(i) = msg.rotor_input[i];
  }
  output->SetFromVector(command);
}

QuadrotorStateReceiver::QuadrotorStateReceiver() {
  input_port_ = this->DeclareAbstractInputPort(
      "lcmt_quadrotor_state", drake::Value<lcmt_quadrotor_state>{})
                    .get_index();
  this->DeclareVectorOutputPort("state", drake::systems::BasicVector<double>(18),
                                &QuadrotorStateReceiver::CopyStateOut);
}

void QuadrotorStateReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& msg =
      this->get_input_port(input_port_).Eval<lcmt_quadrotor_state>(context);
  Eigen::VectorXd state = Eigen::VectorXd::Zero(18);
  for (int i = 0; i < 3; ++i) {
    state(i) = msg.position[i];
    state(i + 3) = msg.velocity[i];
    state(i + 15) = msg.body_angular_velocity[i];
  }
  for (int i = 0; i < 9; ++i) {
    state(i + 6) = msg.rotation[i];
  }
  output->SetFromVector(state);
}

QuadrotorStateSender::QuadrotorStateSender() {
  input_port_ = this->DeclareVectorInputPort(
      "state", drake::systems::BasicVector<double>(18))
                    .get_index();
  this->DeclareAbstractOutputPort("lcmt_quadrotor_state",
                                  &QuadrotorStateSender::CalcMessage);
}

void QuadrotorStateSender::CalcMessage(
    const drake::systems::Context<double>& context,
    lcmt_quadrotor_state* output) const {
  const Eigen::VectorXd value = this->get_input_port(input_port_).Eval(context);
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(
      value.data() + 6);
  const drake::math::RollPitchYaw<double> rpy{
      drake::math::RotationMatrix<double>{R}};
  output->utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  for (int i = 0; i < 3; ++i) {
    output->position[i] = value(i);
    output->velocity[i] = value(i + 3);
    output->rpy[i] = rpy.vector()(i);
    output->body_angular_velocity[i] = value(i + 15);
  }
  for (int i = 0; i < 9; ++i) {
    output->rotation[i] = value(i + 6);
  }
}

QuadrotorCommandSender::QuadrotorCommandSender() {
  input_port_ = this->DeclareVectorInputPort(
      "rotor_input", drake::systems::BasicVector<double>(4))
                    .get_index();
  this->DeclareAbstractOutputPort("lcmt_quadrotor_command",
                                  &QuadrotorCommandSender::CalcMessage);
}

void QuadrotorCommandSender::CalcMessage(
    const drake::systems::Context<double>& context,
    lcmt_quadrotor_command* output) const {
  const Eigen::VectorXd value = this->get_input_port(input_port_).Eval(context);
  output->utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  for (int i = 0; i < 4; ++i) {
    output->rotor_input[i] = value(i);
  }
}

SimTimeSender::SimTimeSender() {
  this->DeclareAbstractOutputPort("sim_time", &SimTimeSender::CalcMessage);
}

void SimTimeSender::CalcMessage(const drake::systems::Context<double>& context,
                                lcmt_sim_time* output) const {
  output->utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  output->time_sec = context.get_time();
}

std::vector<std::string> QuadrotorStateNames() {
  return {"x",  "y",  "z",  "vx", "vy", "vz",
          "R00", "R01", "R02", "R10", "R11", "R12",
          "R20", "R21", "R22", "wx", "wy", "wz"};
}

}  // namespace systems
}  // namespace uav_delivery
