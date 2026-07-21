#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/quadrotor_params.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_reference.hpp"

namespace uav_delivery {
namespace systems {

class Se3Controller final : public drake::systems::LeafSystem<double> {
 public:
  explicit Se3Controller(QuadrotorSimParams params);
  lcmt_quadrotor_reference MakeDefaultReference() const;

 private:
  void CalcCommand(const drake::systems::Context<double>& context,
                   lcmt_quadrotor_command* output) const;

  Eigen::Vector4d MixToRotorInputs(double thrust,
                                   const Eigen::Vector3d& moment_B) const;

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& m);

  QuadrotorSimParams params_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex reference_port_;
};

}  // namespace systems
}  // namespace uav_delivery
