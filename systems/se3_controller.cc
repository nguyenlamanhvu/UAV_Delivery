#include "systems/se3_controller.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "drake/common/value.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "uav_delivery/lcmt_quadrotor_state.hpp"

namespace uav_delivery {
namespace systems {

Se3Controller::Se3Controller(QuadrotorSimParams params)
    : params_(std::move(params)) {
  state_port_ = this->DeclareAbstractInputPort(
      "quadrotor_state", drake::Value<lcmt_quadrotor_state>{})
                    .get_index();
  reference_port_ = this->DeclareAbstractInputPort(
      "quadrotor_reference",
      drake::Value<lcmt_quadrotor_reference>{MakeDefaultReference()})
                    .get_index();
  this->DeclareAbstractOutputPort("lcmt_quadrotor_command",
                                  &Se3Controller::CalcCommand);
}

void Se3Controller::CalcCommand(
    const drake::systems::Context<double>& context,
    lcmt_quadrotor_command* output) const {
  const auto& state =
      this->get_input_port(state_port_).Eval<lcmt_quadrotor_state>(context);
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Matrix3d R;
  Eigen::Vector3d omega_B;
  for (int i = 0; i < 3; ++i) {
    p(i) = state.position[i];
    v(i) = state.velocity[i];
    omega_B(i) = state.body_angular_velocity[i];
  }
  for (int i = 0; i < 9; ++i) {
    R(i / 3, i % 3) = state.rotation[i];
  }

  const auto& reference =
      this->get_input_port(reference_port_).Eval<lcmt_quadrotor_reference>(
          context);
  Eigen::Vector3d pd;
  Eigen::Vector3d vd;
  Eigen::Vector3d ad;
  Eigen::Matrix3d Rd_msg;
  Eigen::Vector3d omega_d_B;
  Eigen::Vector3d alpha_d_B;
  for (int i = 0; i < 3; ++i) {
    pd(i) = reference.position[i];
    vd(i) = reference.velocity[i];
    ad(i) = reference.acceleration[i];
    omega_d_B(i) = reference.body_angular_velocity[i];
    alpha_d_B(i) = reference.body_angular_acceleration[i];
  }
  for (int i = 0; i < 9; ++i) {
    Rd_msg(i / 3, i % 3) = reference.rotation[i];
  }

  const Eigen::Vector3d ep = p - pd;
  const Eigen::Vector3d ev = v - vd;
  const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();

  Eigen::Vector3d desired_force =
      -params_.se3_controller.kp_position * ep -
      params_.se3_controller.kd_position * ev +
      params_.plant.mass * params_.plant.gravity * e3 +
      params_.plant.mass * ad;
  if (desired_force.norm() < 1e-9) {
    desired_force = params_.plant.mass * params_.plant.gravity * e3;
  }

  const Eigen::Vector3d b3d = desired_force.normalized();
  const double desired_yaw = std::atan2(Rd_msg(1, 0), Rd_msg(0, 0));
  const Eigen::Vector3d b1d(std::cos(desired_yaw), std::sin(desired_yaw), 0.0);
  Eigen::Vector3d b2d = b3d.cross(b1d);
  if (b2d.norm() < 1e-9) {
    b2d = Eigen::Vector3d::UnitY();
  } else {
    b2d.normalize();
  }

  Eigen::Matrix3d Rd;
  Rd.col(0) = b2d.cross(b3d);
  Rd.col(1) = b2d;
  Rd.col(2) = b3d;

  const Eigen::Vector3d eR = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
  const Eigen::Vector3d eW = omega_B - omega_d_B;
  const Eigen::Matrix3d J = params_.plant.inertia.asDiagonal();
  const double thrust = desired_force.dot(R * e3);
  const Eigen::Vector3d moment_cmd =
      -params_.se3_controller.kp_rotation * eR -
      params_.se3_controller.kd_angular * eW +
      omega_B.cross(J * omega_B) + J * alpha_d_B;

  const Eigen::Vector4d rotor_input = MixToRotorInputs(thrust, moment_cmd);
  output->utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  for (int i = 0; i < 4; ++i) {
    output->rotor_input[i] = rotor_input(i);
  }
}

Eigen::Vector4d Se3Controller::MixToRotorInputs(
    double thrust, const Eigen::Vector3d& moment_B) const {
  const double b = params_.plant.thrust_coeff;
  const double d = params_.plant.yaw_moment_coeff;
  const double a = params_.plant.thrust_coeff * params_.plant.arm_length /
                   std::sqrt(2.0);

  Eigen::Matrix4d mixer;
  mixer << b, b, b, b,
           a, a, -a, -a,
           -a, a, a, -a,
           d, -d, d, -d;

  Eigen::Vector4d wrench;
  wrench << thrust, moment_B.x(), moment_B.y(), moment_B.z();
  return mixer.fullPivLu().solve(wrench);
}

Eigen::Vector3d Se3Controller::Vee(const Eigen::Matrix3d& m) {
  return {m(2, 1), m(0, 2), m(1, 0)};
}

lcmt_quadrotor_reference Se3Controller::MakeDefaultReference() const {
  lcmt_quadrotor_reference reference{};
  for (int i = 0; i < 3; ++i) {
    reference.position[i] = params_.se3_controller.desired_position(i);
    reference.velocity[i] = params_.se3_controller.desired_velocity(i);
    reference.acceleration[i] = 0.0;
    reference.body_angular_velocity[i] = 0.0;
    reference.body_angular_acceleration[i] = 0.0;
  }
  const Eigen::Matrix3d R =
      drake::math::RollPitchYaw<double>(
          0.0, 0.0, params_.se3_controller.desired_yaw)
          .ToRotationMatrix()
          .matrix();
  for (int i = 0; i < 9; ++i) {
    reference.rotation[i] = R(i / 3, i % 3);
  }
  return reference;
}

}  // namespace systems
}  // namespace uav_delivery
