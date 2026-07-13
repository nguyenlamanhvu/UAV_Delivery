#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/quadrotor_params.h"

namespace uav_delivery {
namespace systems {

class HoverController final : public drake::systems::LeafSystem<double> {
 public:
  explicit HoverController(QuadrotorSimParams params);

 private:
  void CalcCommand(const drake::systems::Context<double>& context,
                   drake::systems::BasicVector<double>* output) const;

  Eigen::Vector4d MixToRotorSpeeds(double thrust,
                                   const Eigen::Vector3d& moment_B) const;

  QuadrotorSimParams params_;
  drake::systems::InputPortIndex state_port_;
};

}  // namespace systems
}  // namespace uav_delivery
