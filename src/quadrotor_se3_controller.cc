#include <csignal>
#include <iostream>
#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "params/quadrotor_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_driven_loop.h"
#include "systems/se3_controller.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"

DEFINE_string(config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_string(diagram_svg, "", "Path or directory for the system diagram SVG.");

namespace uav_delivery {
namespace {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  const QuadrotorSimParams params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  drake::systems::DiagramBuilder<double> builder;
  auto* controller = builder.AddSystem<systems::Se3Controller>(params);
  auto* command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_quadrotor_command>(
          params.lcm_channels.command, &lcm,
          {drake::systems::TriggerType::kForced}));

  builder.Connect(controller->get_output_port(0), command_pub->get_input_port());
  builder.AddSystem<systems::SimTerminator>();

  std::shared_ptr<drake::systems::Diagram<double>> diagram(builder.Build());
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg, argv[0]);

  std::cout << "Quadrotor SE(3) controller config: " << FLAGS_config << "\n";
  std::cout << "Subscribing state on " << params.lcm_channels.state << "\n";
  std::cout << "Publishing command on " << params.lcm_channels.command << "\n";

  systems::LcmDrivenLoop<lcmt_quadrotor_state> loop(
      &lcm, diagram, controller, params.lcm_channels.state,
      true /* force publish once per input state */);
  loop.Simulate(params.sim_time, []() { return systems::g_stop_requested; });
  return 0;
}

}  // namespace
}  // namespace uav_delivery

int main(int argc, char* argv[]) {
  return uav_delivery::DoMain(argc, argv);
}
