#pragma once

#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_sim_time.hpp"
#include "params/quadrotor_params.h"

namespace uav_delivery {
namespace systems {

class QuadrotorCommandReceiver final : public drake::systems::LeafSystem<double> {
 public:
  QuadrotorCommandReceiver();

 private:
  void CopyCommandOut(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex input_port_;
};

class QuadrotorStateReceiver final : public drake::systems::LeafSystem<double> {
 public:
  QuadrotorStateReceiver();

 private:
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex input_port_;
};

class QuadrotorStateSender final : public drake::systems::LeafSystem<double> {
 public:
  QuadrotorStateSender();

 private:
  void CalcMessage(const drake::systems::Context<double>& context,
                   lcmt_quadrotor_state* output) const;

  drake::systems::InputPortIndex input_port_;
};

class QuadrotorCommandSender final : public drake::systems::LeafSystem<double> {
 public:
  QuadrotorCommandSender();

 private:
  void CalcMessage(const drake::systems::Context<double>& context,
                   lcmt_quadrotor_command* output) const;

  drake::systems::InputPortIndex input_port_;
};

class SimTimeSender final : public drake::systems::LeafSystem<double> {
 public:
  SimTimeSender();

 private:
  void CalcMessage(const drake::systems::Context<double>& context,
                   lcmt_sim_time* output) const;
};

std::vector<std::string> QuadrotorStateNames();

}  // namespace systems
}  // namespace uav_delivery
