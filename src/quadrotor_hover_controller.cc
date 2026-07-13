#include <csignal>
#include <iostream>
#include <limits>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "params/quadrotor_params.h"
#include "systems/hover_controller.h"
#include "systems/lcm_systems.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"

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

  const QuadrotorSimParams params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);

  drake::systems::DiagramBuilder<double> builder;
  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);

  auto* state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_state>(
          params.lcm_channels.state, lcm, std::numeric_limits<double>::infinity()));
  auto* state_receiver = builder.AddSystem<systems::QuadrotorStateReceiver>();
  auto* controller = builder.AddSystem<systems::HoverController>(params);
  auto* command_sender = builder.AddSystem<systems::QuadrotorCommandSender>();
  auto* command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_quadrotor_command>(
          params.lcm_channels.command, lcm, 1.0 / params.publish_rate));

  builder.Connect(state_sub->get_output_port(), state_receiver->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0),
                  controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0), command_pub->get_input_port());
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  std::cout << "Quadrotor hover controller config: " << FLAGS_config << "\n";
  std::cout << "Subscribing state on " << params.lcm_channels.state << "\n";
  std::cout << "Publishing command on " << params.lcm_channels.command << "\n";

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
