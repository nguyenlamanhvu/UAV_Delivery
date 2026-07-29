#pragma once

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/moving_target_params.h"

namespace uav_delivery {
namespace systems {

class MovingTargetController final : public drake::systems::LeafSystem<double> {
 public:
  explicit MovingTargetController(MovingTargetControllerParams params);

 private:
  void CalcDriveTorques(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const;

  const MovingTargetControllerParams params_;
  drake::systems::InputPortIndex teleop_port_;
  drake::systems::InputPortIndex state_port_;
};

}  // namespace systems
}  // namespace uav_delivery
