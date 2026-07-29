#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/quadrotor_params.h"
#include "uav_delivery/lcmt_quadrotor_reference.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_timestamped_saved_traj.hpp"

namespace uav_delivery {
namespace systems {

class ReferenceTrajectoryUnpacker final
    : public drake::systems::LeafSystem<double> {
 public:
  explicit ReferenceTrajectoryUnpacker(QuadrotorSimParams params);
  lcmt_timestamped_saved_traj MakeDefaultTrajectory() const;

 private:
  void CalcReference(const drake::systems::Context<double>& context,
                     lcmt_quadrotor_reference* output) const;
  lcmt_quadrotor_reference MakeDefaultReference(double time) const;

  QuadrotorSimParams params_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex trajectory_port_;
};

}  // namespace systems
}  // namespace uav_delivery
