#include "systems/quadrotor_plant.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace uav_delivery {
namespace systems {

QuadrotorPlant::QuadrotorPlant(QuadrotorParams params) : params_(std::move(params)) {
  command_port_ = this->DeclareVectorInputPort(
      "rotor_input", drake::systems::BasicVector<double>(4))
                      .get_index();
  this->DeclareContinuousState(kStateSize);
  this->DeclareVectorOutputPort("state",
                                drake::systems::BasicVector<double>(kStateSize),
                                &QuadrotorPlant::CopyState);
}

const drake::systems::InputPort<double>& QuadrotorPlant::get_command_input_port()
    const {
  return this->get_input_port(command_port_);
}

void QuadrotorPlant::DoCalcTimeDerivatives(
    const drake::systems::Context<double>& context,
    drake::systems::ContinuousState<double>* derivatives) const {
  const Eigen::VectorXd x = context.get_continuous_state_vector().CopyToVector();
  const Eigen::Vector3d velocity_W = x.segment<3>(3);
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_WB(
      x.data() + 6);
  const Eigen::Vector3d omega_B = x.segment<3>(15);

  const Eigen::VectorXd command = this->get_command_input_port().Eval(context);
  Eigen::Vector4d rotor_inputs = command.head<4>();
  for (int i = 0; i < rotor_inputs.size(); ++i) {
    rotor_inputs(i) = std::clamp(rotor_inputs(i), 0.0, params_.max_rotor_input);
  }

  const RotorWrench wrench = CalcRotorWrench(rotor_inputs);
  const Eigen::Vector3d force_W =
      R_WB * Eigen::Vector3d(0.0, 0.0, wrench.total_thrust) -
      Eigen::Vector3d(0.0, 0.0, params_.mass * params_.gravity);

  Eigen::VectorXd xdot = Eigen::VectorXd::Zero(kStateSize);
  xdot.segment<3>(0) = velocity_W;
  xdot.segment<3>(3) = force_W / params_.mass;
  const Eigen::Matrix3d Rdot = R_WB * Hat(omega_B);
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(xdot.data() + 6) =
      Rdot;

  const Eigen::Matrix3d I = params_.inertia.asDiagonal();
  xdot.segment<3>(15) =
      I.inverse() * (wrench.moment_B - omega_B.cross(I * omega_B));
  derivatives->SetFromVector(xdot);
}

void QuadrotorPlant::CopyState(const drake::systems::Context<double>& context,
                               drake::systems::BasicVector<double>* output) const {
  output->SetFromVector(context.get_continuous_state_vector().CopyToVector());
}

QuadrotorPlant::RotorWrench QuadrotorPlant::CalcRotorWrench(
    const Eigen::Vector4d& rotor_inputs) const {
  const double b = params_.thrust_coeff;
  const double d = params_.yaw_moment_coeff;
  const double a = params_.thrust_coeff * params_.arm_length / std::sqrt(2.0);
  const double f1 = b * rotor_inputs(0);
  const double f2 = b * rotor_inputs(1);
  const double f3 = b * rotor_inputs(2);
  const double f4 = b * rotor_inputs(3);

  RotorWrench wrench;
  wrench.total_thrust = f1 + f2 + f3 + f4;
  wrench.moment_B << a * (rotor_inputs(0) + rotor_inputs(1) -
                          rotor_inputs(2) - rotor_inputs(3)),
      a * (-rotor_inputs(0) + rotor_inputs(1) + rotor_inputs(2) -
           rotor_inputs(3)),
      d * (rotor_inputs(0) - rotor_inputs(1) + rotor_inputs(2) -
           rotor_inputs(3));
  return wrench;
}

Eigen::Matrix3d QuadrotorPlant::Hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

}  // namespace systems
}  // namespace uav_delivery
