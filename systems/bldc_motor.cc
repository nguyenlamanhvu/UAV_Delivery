#include "systems/bldc_motor.h"

namespace uav_delivery {
namespace systems {

BldcMotor::BldcMotor(const MotorParams& params) : tau_(params.time_constant) {
  // Input: 4 motor throttle commands
  this->DeclareVectorInputPort("command_input", drake::systems::BasicVector<double>(4));
  
  // State: 4 actual motor throttle states
  this->DeclareContinuousState(4);
  
  // Output: 4 actual motor throttle states
  this->DeclareVectorOutputPort("actual_throttle", drake::systems::BasicVector<double>(4),
                                &BldcMotor::CalcOutput);
}

void BldcMotor::DoCalcTimeDerivatives(
    const drake::systems::Context<double>& context,
    drake::systems::ContinuousState<double>* derivatives) const {
  const auto& u = this->get_input_port(0).Eval(context);
  const auto& x = context.get_continuous_state_vector();
  auto& x_dot = derivatives->get_mutable_vector();

  for (int i = 0; i < 4; ++i) {
    x_dot[i] = (u[i] - x[i]) / tau_;
  }
}

void BldcMotor::CalcOutput(const drake::systems::Context<double>& context,
                           drake::systems::BasicVector<double>* output) const {
  const auto& x = context.get_continuous_state_vector();
  output->SetFromVector(x.CopyToVector());
}

}  // namespace systems
}  // namespace uav_delivery
