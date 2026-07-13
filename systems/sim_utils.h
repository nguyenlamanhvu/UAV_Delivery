#pragma once

#include <csignal>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"

namespace uav_delivery {
namespace systems {

extern volatile std::sig_atomic_t g_stop_requested;

void HandleSigint(int);

class ConsoleLogger final : public drake::systems::LeafSystem<double> {
 public:
  ConsoleLogger(int state_size, double period);

 private:
  drake::systems::EventStatus Print(
      const drake::systems::Context<double>& context) const;

  drake::systems::InputPortIndex input_port_;
};

class SimTerminator final : public drake::systems::LeafSystem<double> {
 public:
  SimTerminator();

 private:
  drake::systems::EventStatus CheckForStop(
      const drake::systems::Context<double>&,
      drake::systems::State<double>*) const;
};

}  // namespace systems
}  // namespace uav_delivery
