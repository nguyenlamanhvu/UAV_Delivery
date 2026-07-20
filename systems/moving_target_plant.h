#pragma once

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/moving_target_params.h"

namespace uav_delivery {
namespace systems {

class MovingTargetPlant final : public drake::systems::LeafSystem<double> {
 public:
  explicit MovingTargetPlant(MovingTargetPlantParams params);

  static constexpr int kStateSize = 9;

  const drake::systems::InputPort<double>& get_command_input_port() const;

 private:
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override;

  void CopyState(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* output) const;

  const MovingTargetPlantParams params_;
  drake::systems::InputPortIndex command_port_;
};

}  // namespace systems
}  // namespace uav_delivery
