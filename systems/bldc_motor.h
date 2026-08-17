#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "params/quadrotor_params.h"

namespace uav_delivery {
namespace systems {

class BldcMotor : public drake::systems::LeafSystem<double> {
 public:
  explicit BldcMotor(const MotorParams& params);

 private:
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override;

  void CalcOutput(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const;

  double tau_;
};

}  // namespace systems
}  // namespace uav_delivery
