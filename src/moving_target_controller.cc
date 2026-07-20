#include <algorithm>
#include <csignal>
#include <iostream>
#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "params/moving_target_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_driven_loop.h"
#include "systems/moving_target_controller.h"
#include "systems/moving_target_lcm_systems.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_moving_target_actuation_command.hpp"
#include "uav_delivery/lcmt_moving_target_teleop_command.hpp"

DEFINE_string(config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_string(diagram_svg, "", "Optional path to write the system diagram SVG.");

namespace uav_delivery {
namespace {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  const MovingTargetSimParams params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_config);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  drake::systems::DiagramBuilder<double> builder;
  auto* teleop_receiver = builder.AddSystem<systems::MovingTargetTeleopReceiver>();
  auto* controller =
      builder.AddSystem<systems::MovingTargetController>(params.controller);
  auto* actuation_sender =
      builder.AddSystem<systems::MovingTargetActuationSender>();
  auto* actuation_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<
          lcmt_moving_target_actuation_command>(
          params.lcm_channels.actuation_command, &lcm,
          {drake::systems::TriggerType::kForced}));

  builder.Connect(teleop_receiver->get_output_port(0),
                  controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0),
                  actuation_sender->get_input_port(0));
  builder.Connect(actuation_sender->get_output_port(0),
                  actuation_pub->get_input_port());
  builder.AddSystem<systems::SimTerminator>();

  std::shared_ptr<drake::systems::Diagram<double>> diagram(builder.Build());
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg);

  std::cout << "moving_target controller config: " << FLAGS_config << "\n";
  std::cout << "Subscribing teleop on " << params.lcm_channels.teleop_command << "\n";
  std::cout << "Publishing actuation on " << params.lcm_channels.actuation_command
            << "\n";

  systems::LcmDrivenLoop<lcmt_moving_target_teleop_command> loop(
      &lcm, diagram, teleop_receiver, params.lcm_channels.teleop_command,
      true /* force publish once per input */);
  loop.Simulate(params.sim_time, []() { return systems::g_stop_requested; });
  return 0;
}

}  // namespace
}  // namespace uav_delivery

int main(int argc, char* argv[]) {
  return uav_delivery::DoMain(argc, argv);
}
