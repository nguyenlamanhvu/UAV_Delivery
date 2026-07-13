#include <csignal>
#include <iostream>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "params/quadrotor_params.h"
#include "systems/lcm_systems.h"
#include "systems/quadrotor_plant.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_sim_time.hpp"

DEFINE_string(config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");

namespace uav_delivery {
namespace {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  QuadrotorSimParams params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);

  drake::systems::DiagramBuilder<double> builder;
  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto* quadrotor = builder.AddSystem<systems::QuadrotorPlant>(params.plant);

  auto* command_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_command>(
          params.lcm_channels.command, lcm));
  auto* command_receiver =
      builder.AddSystem<systems::QuadrotorCommandReceiver>();
  builder.Connect(command_sub->get_output_port(), command_receiver->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(0),
                  quadrotor->get_command_input_port());

  auto* state_sender = builder.AddSystem<systems::QuadrotorStateSender>();
  builder.Connect(quadrotor->get_output_port(0), state_sender->get_input_port(0));
  auto* state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_quadrotor_state>(
          params.lcm_channels.state, lcm, 1.0 / params.publish_rate));
  builder.Connect(state_sender->get_output_port(0), state_pub->get_input_port());

  auto* sim_time_sender = builder.AddSystem<systems::SimTimeSender>();
  auto* sim_time_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_sim_time>(
          params.lcm_channels.sim_time, lcm, 1.0 / params.publish_rate));
  builder.Connect(sim_time_sender->get_output_port(0), sim_time_pub->get_input_port());

  if (params.console_log) {
    auto* console_logger = builder.AddSystem<systems::ConsoleLogger>(
        systems::QuadrotorPlant::kStateSize, params.console_period);
    builder.Connect(quadrotor->get_output_port(0), console_logger->get_input_port(0));
  }
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& quad_context = diagram->GetMutableSubsystemContext(*quadrotor, context.get());
  quad_context.SetContinuousState(MakeInitialStateVector(params.initial_state));

  std::cout << "Quadrotor sim config: " << FLAGS_config << "\n";
  std::cout << "Publishing state on " << params.lcm_channels.state
            << " and sim time on " << params.lcm_channels.sim_time << "\n";
  std::cout << "Receiving rotor commands on " << params.lcm_channels.command
            << " as lcmt_quadrotor_command [w1,w2,w3,w4] rad/s\n";
  std::cout << "Debug with: drake-lcm-spy or lcm-spy" << std::endl;

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(params.realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(params.sim_time);
  return 0;
}

}  // namespace
}  // namespace uav_delivery

int main(int argc, char* argv[]) {
  return uav_delivery::DoMain(argc, argv);
}
