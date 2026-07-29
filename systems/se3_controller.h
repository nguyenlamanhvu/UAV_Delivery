#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/quadrotor_params.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_setpoint.hpp"

namespace uav_delivery {
namespace systems {

class Se3Controller final : public drake::systems::LeafSystem<double> {
 public:
  explicit Se3Controller(QuadrotorSimParams params);

 private:
  struct ResolvedSetpoint {
    int mode{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
  };

  struct ControlSolution {
    Eigen::Vector3d desired_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d force_required_W{Eigen::Vector3d::Zero()};
    Eigen::Vector3d body_moment_cmd{Eigen::Vector3d::Zero()};
    Eigen::Vector4d rotor_input{Eigen::Vector4d::Zero()};
    double collective_thrust{0.0};
    bool saturated{false};
  };

  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override;

  void CalcCommand(const drake::systems::Context<double>& context,
                   lcmt_quadrotor_command* output) const;

  ResolvedSetpoint ResolveSetpoint(
      const drake::systems::Context<double>& context) const;
  ControlSolution SolveControl(
      const drake::systems::Context<double>& context,
      const Eigen::Vector3d& integral_velocity_error,
      const Eigen::Vector3d& momentum_observer_state) const;
  Eigen::Vector4d AllocateRotorInputs(double thrust,
                                      const Eigen::Vector3d& moment_B,
                                      bool* saturated) const;
  Eigen::Vector4d SolveMixer(double thrust,
                             const Eigen::Vector3d& moment_B) const;

  static Eigen::Vector3d ClampNorm(const Eigen::Vector3d& value, double max_norm);

  QuadrotorSimParams params_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex setpoint_port_;
};

}  // namespace systems
}  // namespace uav_delivery
