#pragma once

#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "uav_delivery/lcmt_moving_target_actuation_command.hpp"
#include "uav_delivery/lcmt_moving_target_state.hpp"
#include "uav_delivery/lcmt_moving_target_teleop_command.hpp"

namespace uav_delivery {
namespace systems {

class MovingTargetTeleopReceiver final : public drake::systems::LeafSystem<double> {
 public:
  MovingTargetTeleopReceiver();

 private:
  void CopyTeleopOut(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex input_port_;
};

class MovingTargetActuationReceiver final : public drake::systems::LeafSystem<double> {
 public:
  MovingTargetActuationReceiver();

 private:
  void CopyActuationOut(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex input_port_;
};

class MovingTargetStateReceiver final : public drake::systems::LeafSystem<double> {
 public:
  MovingTargetStateReceiver();

 private:
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex input_port_;
};

class MovingTargetActuationSender final : public drake::systems::LeafSystem<double> {
 public:
  MovingTargetActuationSender();

 private:
  void CalcMessage(const drake::systems::Context<double>& context,
                   lcmt_moving_target_actuation_command* output) const;

  drake::systems::InputPortIndex input_port_;
};

class MovingTargetStateSender final : public drake::systems::LeafSystem<double> {
 public:
  MovingTargetStateSender();

 private:
  void CalcMessage(const drake::systems::Context<double>& context,
                   lcmt_moving_target_state* output) const;

  drake::systems::InputPortIndex input_port_;
};

std::vector<std::string> MovingTargetStateNames();

}  // namespace systems
}  // namespace uav_delivery
