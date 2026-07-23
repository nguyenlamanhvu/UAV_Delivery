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
#include "params/moving_target_params.h"
#include "params/quadrotor_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_systems.h"
#include "systems/moving_target_controller.h"
#include "systems/moving_target_lcm_systems.h"
#include "systems/moving_target_plant.h"
#include "systems/quadrotor_plant.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_moving_target_state.hpp"
#include "uav_delivery/lcmt_moving_target_teleop_command.hpp"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_sim_time.hpp"

DEFINE_string(config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(moving_target_config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams for the car inside "
              "the quadrotor environment.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_string(diagram_svg, "", "Optional path to write the system diagram SVG.");

namespace uav_delivery {
namespace {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  QuadrotorSimParams params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);
  const MovingTargetSimParams moving_target_params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_moving_target_config);

  drake::systems::DiagramBuilder<double> builder;
  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto* quadrotor = builder.AddSystem<systems::QuadrotorPlant>(params.plant);
  auto* moving_target =
      builder.AddSystem<systems::MovingTargetPlant>(moving_target_params.plant);

  auto* command_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_command>(
          params.lcm_channels.command, lcm));
  auto* command_receiver =
      builder.AddSystem<systems::QuadrotorCommandReceiver>();
  builder.Connect(command_sub->get_output_port(), command_receiver->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(0),
                  quadrotor->get_command_input_port());

  auto* moving_target_teleop_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          lcmt_moving_target_teleop_command>(
          moving_target_params.lcm_channels.teleop_command, lcm));
  auto* moving_target_teleop_receiver =
      builder.AddSystem<systems::MovingTargetTeleopReceiver>();
  auto* moving_target_controller = builder.AddSystem<systems::MovingTargetController>(
      moving_target_params.controller);
  builder.Connect(moving_target_teleop_sub->get_output_port(),
                  moving_target_teleop_receiver->get_input_port(0));
  builder.Connect(moving_target_teleop_receiver->get_output_port(0),
                  moving_target_controller->get_input_port(0));
  builder.Connect(moving_target->get_output_port(0),
                  moving_target_controller->get_input_port(1));
  builder.Connect(moving_target_controller->get_output_port(0),
                  moving_target->get_command_input_port());

  auto* state_sender = builder.AddSystem<systems::QuadrotorStateSender>();
  builder.Connect(quadrotor->get_output_port(0), state_sender->get_input_port(0));
  auto* state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_quadrotor_state>(
          params.lcm_channels.state, lcm, 1.0 / params.publish_rate));
  builder.Connect(state_sender->get_output_port(0), state_pub->get_input_port());

  auto* moving_target_state_sender =
      builder.AddSystem<systems::MovingTargetStateSender>();
  builder.Connect(moving_target->get_output_port(0),
                  moving_target_state_sender->get_input_port(0));
  auto* moving_target_state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_moving_target_state>(
          moving_target_params.lcm_channels.state, lcm,
          1.0 / moving_target_params.publish_rate));
  builder.Connect(moving_target_state_sender->get_output_port(0),
                  moving_target_state_pub->get_input_port());

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
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg);
  auto context = diagram->CreateDefaultContext();
  auto& quad_context = diagram->GetMutableSubsystemContext(*quadrotor, context.get());
  quad_context.SetContinuousState(MakeInitialStateVector(params.initial_state));
  auto& moving_target_context =
      diagram->GetMutableSubsystemContext(*moving_target, context.get());
  moving_target_context.SetContinuousState(
      MakeMovingTargetInitialStateVector(moving_target_params.initial_state));

  std::cout << "Quadrotor sim config: " << FLAGS_config << "\n";
  std::cout << "Moving target config: " << FLAGS_moving_target_config << "\n";
  std::cout << "Quadrotor model: " << params.model << "\n";
  std::cout << "Moving target model: " << moving_target_params.model << "\n";
  std::cout << "Publishing state on " << params.lcm_channels.state
            << " and sim time on " << params.lcm_channels.sim_time << "\n";
  std::cout << "Receiving propeller inputs on " << params.lcm_channels.command
            << " as lcmt_quadrotor_command [u1,u2,u3,u4]\n";
  std::cout << "Receiving moving target teleop on "
            << moving_target_params.lcm_channels.teleop_command << "\n";
  std::cout << "Publishing moving target state on "
            << moving_target_params.lcm_channels.state << "\n";
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
