#include "systems/se3_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "drake/common/value.h"
#include "uav_delivery/lcmt_quadrotor_state.hpp"

namespace uav_delivery {
namespace systems {
namespace {

constexpr int kModePosition = 0;
constexpr int kModeVelocity = 1;
constexpr int kModeDirect = 2;

}  // namespace

Se3Controller::Se3Controller(QuadrotorSimParams params)
    : params_(std::move(params)) {
  state_port_ = this->DeclareAbstractInputPort(
      "quadrotor_state", drake::Value<lcmt_quadrotor_state>{})
                    .get_index();
  setpoint_port_ = this->DeclareAbstractInputPort(
      "quadrotor_setpoint", drake::Value<lcmt_quadrotor_setpoint>{})
                       .get_index();
  this->DeclareContinuousState(6);
  this->DeclareAbstractOutputPort("lcmt_quadrotor_command",
                                  &Se3Controller::CalcCommand);
}

void Se3Controller::DoCalcTimeDerivatives(
    const drake::systems::Context<double>& context,
    drake::systems::ContinuousState<double>* derivatives) const {
  const Eigen::VectorXd z = context.get_continuous_state_vector().CopyToVector();
  const Eigen::Vector3d integral_velocity_error = z.segment<3>(0);
  const Eigen::Vector3d momentum_observer_state = z.segment<3>(3);
  const ControlSolution solution =
      SolveControl(context, integral_velocity_error, momentum_observer_state);

  const auto& state =
      this->get_input_port(state_port_).Eval<lcmt_quadrotor_state>(context);
  Eigen::Vector3d velocity_W;
  Eigen::Matrix3d R_WB;
  for (int i = 0; i < 3; ++i) {
    velocity_W(i) = state.velocity[i];
  }
  for (int i = 0; i < 9; ++i) {
    R_WB(i / 3, i % 3) = state.rotation[i];
  }

  const Eigen::Vector3d actual_momentum = params_.plant.mass * velocity_W;
  const Eigen::Vector3d body_z_W = R_WB * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d p_hat_dot =
      -params_.se3_controller.observer_gain *
          (momentum_observer_state - actual_momentum) +
      -params_.plant.mass * params_.plant.gravity * Eigen::Vector3d::UnitZ() +
      body_z_W * solution.collective_thrust;

  Eigen::Vector3d I_v_dot = Eigen::Vector3d::Zero();
  if (!solution.saturated &&
      velocity_W.norm() <= params_.se3_controller.max_integral_speed) {
    I_v_dot = velocity_W - solution.desired_velocity;
  }

  Eigen::VectorXd zdot(6);
  zdot << I_v_dot, p_hat_dot;
  derivatives->SetFromVector(zdot);
}

void Se3Controller::CalcCommand(
    const drake::systems::Context<double>& context,
    lcmt_quadrotor_command* output) const {
  const Eigen::VectorXd z = context.get_continuous_state_vector().CopyToVector();
  const ControlSolution solution =
      SolveControl(context, z.segment<3>(0), z.segment<3>(3));

  output->utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  for (int i = 0; i < 4; ++i) {
    output->rotor_input[i] = solution.rotor_input(i);
  }
}

Se3Controller::ResolvedSetpoint Se3Controller::ResolveSetpoint(
    const drake::systems::Context<double>& context) const {
  ResolvedSetpoint resolved;
  resolved.mode = kModePosition;
  resolved.position = params_.se3_controller.desired_position;
  resolved.velocity = params_.se3_controller.desired_velocity;
  resolved.acceleration = Eigen::Vector3d::Zero();
  resolved.yaw = params_.se3_controller.desired_yaw;
  resolved.yaw_rate = 0.0;

  const auto& setpoint =
      this->get_input_port(setpoint_port_).Eval<lcmt_quadrotor_setpoint>(context);
  const bool has_dynamic_setpoint =
      setpoint.utime != 0 || setpoint.mode != 0 ||
      std::abs(setpoint.yaw) > 1e-12 || std::abs(setpoint.yaw_rate) > 1e-12;
  if (!has_dynamic_setpoint) {
    return resolved;
  }

  resolved.mode = setpoint.mode;
  for (int i = 0; i < 3; ++i) {
    resolved.position(i) = setpoint.position[i];
    resolved.velocity(i) = setpoint.velocity[i];
    resolved.acceleration(i) = setpoint.acceleration[i];
  }
  resolved.yaw = setpoint.yaw;
  resolved.yaw_rate = setpoint.yaw_rate;
  return resolved;
}

Se3Controller::ControlSolution Se3Controller::SolveControl(
    const drake::systems::Context<double>& context,
    const Eigen::Vector3d& integral_velocity_error,
    const Eigen::Vector3d& momentum_observer_state) const {
  const auto& state =
      this->get_input_port(state_port_).Eval<lcmt_quadrotor_state>(context);
  const ResolvedSetpoint setpoint = ResolveSetpoint(context);

  Eigen::Vector3d position_W;
  Eigen::Vector3d velocity_W;
  Eigen::Matrix3d R_WB;
  Eigen::Vector3d omega_B;
  for (int i = 0; i < 3; ++i) {
    position_W(i) = state.position[i];
    velocity_W(i) = state.velocity[i];
    omega_B(i) = state.body_angular_velocity[i];
  }
  for (int i = 0; i < 9; ++i) {
    R_WB(i / 3, i % 3) = state.rotation[i];
  }

  const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d desired_velocity = setpoint.velocity;
  if (setpoint.mode == kModePosition) {
    const Eigen::Vector3d position_error = position_W - setpoint.position;
    desired_velocity -= params_.se3_controller.kp_position * position_error;
  } else if (setpoint.mode == kModeDirect) {
    desired_velocity = setpoint.velocity;
  }
  desired_velocity = ClampNorm(desired_velocity, params_.se3_controller.max_velocity);

  const Eigen::Vector3d velocity_error = velocity_W - desired_velocity;
  const Eigen::Vector3d velocity_body = R_WB.transpose() * velocity_W;
  const Eigen::Vector3d drag_force_W =
      R_WB * params_.se3_controller.aerodynamic_drag.cwiseProduct(velocity_body);
  const Eigen::Vector3d actual_momentum = params_.plant.mass * velocity_W;
  Eigen::Vector3d disturbance_force =
      params_.se3_controller.observer_force_gain *
      (actual_momentum - momentum_observer_state);
  disturbance_force = ClampNorm(disturbance_force,
                                params_.se3_controller.max_observer_force);

  Eigen::Vector3d required_force_W =
      -params_.se3_controller.kv_velocity.cwiseProduct(velocity_error) -
      params_.se3_controller.ki_velocity.cwiseProduct(integral_velocity_error) +
      params_.plant.mass * params_.plant.gravity * e3 +
      params_.plant.mass * setpoint.acceleration + drag_force_W -
      disturbance_force;
      
  // Prevent falling due to negative Z force demand
  if (required_force_W.z() < 0.1) {
    required_force_W.z() = 0.1;
  }
  
  // Enforce maximum tilt angle of ~40 degrees (tan(40) = 0.839)
  const double max_tilt_tan = 0.839;
  double f_xy_norm = required_force_W.head<2>().norm();
  if (f_xy_norm > max_tilt_tan * required_force_W.z()) {
    required_force_W.head<2>() *= (max_tilt_tan * required_force_W.z() / f_xy_norm);
  }
  
  if (required_force_W.norm() < 1e-9) {
    required_force_W = params_.plant.mass * params_.plant.gravity * e3;
  }

  const Eigen::Vector3d body_z_W = R_WB * e3;
  const double collective_thrust = std::max(0.0, required_force_W.dot(body_z_W));

  const Eigen::Vector3d b3d = required_force_W.normalized();
  const Eigen::Vector3d roll_pitch_error_W = body_z_W.cross(b3d);
  const Eigen::Vector3d roll_pitch_error_B = R_WB.transpose() * roll_pitch_error_W;

  const Eigen::Vector3d body_x_W = R_WB.col(0);
  const Eigen::Vector3d body_y_W = R_WB.col(1);
  const Eigen::Vector3d b1d(std::cos(setpoint.yaw), std::sin(setpoint.yaw), 0.0);
  (void)body_x_W;
  const double yaw_error = -b1d.dot(body_y_W);

  Eigen::Vector3d desired_omega_B = Eigen::Vector3d::Zero();
  desired_omega_B.head<2>() =
      params_.se3_controller.k_rp.head<2>().cwiseProduct(
          roll_pitch_error_B.head<2>());
  desired_omega_B.z() = -params_.se3_controller.k_yaw * yaw_error +
                        setpoint.yaw_rate;
  const Eigen::Vector3d omega_error = omega_B - desired_omega_B;

  const Eigen::Matrix3d J = params_.plant.inertia.asDiagonal();
  const Eigen::Vector3d moment_cmd =
      -params_.se3_controller.k_omega.cwiseProduct(omega_error) +
      omega_B.cross(J * omega_B);

  bool saturated = false;
  const Eigen::Vector4d rotor_input =
      AllocateRotorInputs(collective_thrust, moment_cmd, &saturated);

  ControlSolution solution;
  solution.desired_velocity = desired_velocity;
  solution.force_required_W = required_force_W;
  solution.body_moment_cmd = moment_cmd;
  solution.rotor_input = rotor_input;
  solution.collective_thrust = collective_thrust;
  solution.saturated = saturated;
  return solution;
}

Eigen::Vector4d Se3Controller::AllocateRotorInputs(
    double thrust, const Eigen::Vector3d& moment_B, bool* saturated) const {
  
  // 1. Compute required differential thrusts for the moment
  Eigen::Vector4d diff_u = SolveMixer(0.0, moment_B);
  
  // Check how much margin we need for the differential thrust
  double max_diff_u = diff_u.maxCoeff();
  double min_diff_u = diff_u.minCoeff(); // Usually negative
  
  // If the commanded moment requires more differential thrust than the motors can physically
  // provide even if base_u was perfectly centered, we must scale down the moment.
  double diff_span = max_diff_u - min_diff_u;
  double alpha = 1.0;
  if (diff_span > params_.plant.max_rotor_input) {
    alpha = params_.plant.max_rotor_input / diff_span;
    diff_u *= alpha;
    *saturated = true;
    max_diff_u *= alpha;
    min_diff_u *= alpha;
  }
  
  // 2. Compute base thrust
  double base_u = thrust / (4.0 * params_.plant.thrust_coeff);
  
  // 3. Prioritize attitude! Shift base_u so that base_u + diff_u fits in [0, max_rotor_input]
  // We must ensure: base_u + max_diff_u <= max_rotor_input  =>  base_u <= max_rotor_input - max_diff_u
  // We must ensure: base_u + min_diff_u >= 0                =>  base_u >= -min_diff_u
  
  double max_allowed_base_u = params_.plant.max_rotor_input - max_diff_u;
  double min_allowed_base_u = -min_diff_u;
  
  // Clamp base_u to respect the attitude margins
  if (base_u > max_allowed_base_u) {
    base_u = max_allowed_base_u;
    *saturated = true;
  }
  if (base_u < min_allowed_base_u) {
    base_u = min_allowed_base_u;
    *saturated = true;
  }
  
  // 4. Combine
  Eigen::Vector4d rotor_input = Eigen::Vector4d::Constant(base_u) + diff_u;
  
  // Final safety clamp for floating point inaccuracies
  for (int i = 0; i < 4; ++i) {
    rotor_input(i) = std::clamp(rotor_input(i), 0.0, params_.plant.max_rotor_input);
  }
  
  return rotor_input;
}

Eigen::Vector4d Se3Controller::SolveMixer(
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

Eigen::Vector3d Se3Controller::ClampNorm(const Eigen::Vector3d& value,
                                         double max_norm) {
  if (max_norm <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double norm = value.norm();
  if (norm <= max_norm || norm <= std::numeric_limits<double>::epsilon()) {
    return value;
  }
  return value * (max_norm / norm);
}

}  // namespace systems
}  // namespace uav_delivery
