#include "systems/quadrotor_plant.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace uav_delivery {
namespace systems {

QuadrotorPlant::QuadrotorPlant(QuadrotorParams params) : params_(std::move(params)) {
  command_port_ = this->DeclareVectorInputPort(
      "rotor_speed_command_rad_per_sec", drake::systems::BasicVector<double>(4))
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
  const Eigen::Vector3d rpy = x.segment<3>(3);
  const Eigen::Vector3d velocity_W = x.segment<3>(6);
  const Eigen::Vector3d omega_B = x.segment<3>(9);

  const Eigen::VectorXd command = this->get_command_input_port().Eval(context);
  Eigen::Vector4d rotor_speeds = command.head<4>();
  for (int i = 0; i < rotor_speeds.size(); ++i) {
    rotor_speeds(i) = std::clamp(rotor_speeds(i), 0.0, params_.max_rotor_speed);
  }

  const RotorWrench wrench = CalcRotorWrench(rotor_speeds);
  const Eigen::Matrix3d R_WB = RotationMatrixFromRpy(rpy);
  const Eigen::Vector3d force_W =
      R_WB * Eigen::Vector3d(0.0, 0.0, wrench.total_thrust) -
      Eigen::Vector3d(0.0, 0.0, params_.mass * params_.gravity);

  Eigen::VectorXd xdot = Eigen::VectorXd::Zero(kStateSize);
  xdot.segment<3>(0) = velocity_W;
  xdot.segment<3>(3) = EulerRatesFromBodyRates(rpy, omega_B);
  xdot.segment<3>(6) = force_W / params_.mass;

  const Eigen::Matrix3d I = params_.inertia.asDiagonal();
  xdot.segment<3>(9) =
      I.inverse() * (wrench.moment_B - omega_B.cross(I * omega_B));
  derivatives->SetFromVector(xdot);
}

void QuadrotorPlant::CopyState(const drake::systems::Context<double>& context,
                               drake::systems::BasicVector<double>* output) const {
  output->SetFromVector(context.get_continuous_state_vector().CopyToVector());
}

QuadrotorPlant::RotorWrench QuadrotorPlant::CalcRotorWrench(
    const Eigen::Vector4d& rotor_speeds) const {
  const Eigen::Array4d omega_squared = rotor_speeds.array().square();
  const double b = params_.thrust_coeff;
  const double d = params_.yaw_moment_coeff;
  const double l = params_.arm_length;
  const double f1 = b * omega_squared(0);
  const double f2 = b * omega_squared(1);
  const double f3 = b * omega_squared(2);
  const double f4 = b * omega_squared(3);

  RotorWrench wrench;
  wrench.total_thrust = f1 + f2 + f3 + f4;
  wrench.moment_B << l * (f2 - f4), l * (f3 - f1),
      d * (omega_squared(0) - omega_squared(1) + omega_squared(2) -
           omega_squared(3));
  return wrench;
}

Eigen::Matrix3d QuadrotorPlant::RotationMatrixFromRpy(const Eigen::Vector3d& rpy) {
  const double cr = std::cos(rpy.x());
  const double sr = std::sin(rpy.x());
  const double cp = std::cos(rpy.y());
  const double sp = std::sin(rpy.y());
  const double cy = std::cos(rpy.z());
  const double sy = std::sin(rpy.z());

  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp,
      sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr;
  return R;
}

Eigen::Vector3d QuadrotorPlant::EulerRatesFromBodyRates(
    const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega_B) {
  const double roll = rpy.x();
  const double pitch = rpy.y();
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double tp = std::tan(pitch);
  const double cp = std::cos(pitch);

  Eigen::Matrix3d E;
  E << 1.0, sr * tp, cr * tp, 0.0, cr, -sr, 0.0, sr / cp, cr / cp;
  return E * omega_B;
}

}  // namespace systems
}  // namespace uav_delivery
