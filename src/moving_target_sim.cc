#include <csignal>
#include <iostream>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "params/moving_target_params.h"
#include "systems/diagram_utils.h"
#include "systems/moving_target_lcm_systems.h"
#include "systems/moving_target_plant.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_moving_target_actuation_command.hpp"
#include "uav_delivery/lcmt_moving_target_state.hpp"

DEFINE_string(config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_string(diagram_svg, "", "Optional path to write the system diagram SVG.");

namespace uav_delivery {
namespace {

class MovingTargetConsoleLogger final : public drake::systems::LeafSystem<double> {
 public:
  explicit MovingTargetConsoleLogger(double period) {
    input_port_ = this->DeclareVectorInputPort(
        "state", drake::systems::BasicVector<double>(systems::MovingTargetPlant::kStateSize))
                      .get_index();
    this->DeclarePeriodicPublishEvent(period, 0.0,
                                      &MovingTargetConsoleLogger::Print);
  }

 private:
  drake::systems::EventStatus Print(
      const drake::systems::Context<double>& context) const {
    const Eigen::VectorXd x = this->get_input_port(input_port_).Eval(context);
    std::cout << "t=" << context.get_time() << " x=" << x(0) << " y=" << x(1)
              << " yaw=" << x(2) << " v=" << x(3)
              << " yaw_rate=" << x(4) << "\n";
    return drake::systems::EventStatus::Succeeded();
  }

  drake::systems::InputPortIndex input_port_;
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  const MovingTargetSimParams params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_config);

  drake::systems::DiagramBuilder<double> builder;
  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto* moving_target =
      builder.AddSystem<systems::MovingTargetPlant>(params.plant);

  auto* actuation_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          lcmt_moving_target_actuation_command>(
          params.lcm_channels.actuation_command, lcm));
  auto* actuation_receiver =
      builder.AddSystem<systems::MovingTargetActuationReceiver>();
  builder.Connect(actuation_sub->get_output_port(),
                  actuation_receiver->get_input_port(0));
  builder.Connect(actuation_receiver->get_output_port(0),
                  moving_target->get_command_input_port());

  auto* state_sender = builder.AddSystem<systems::MovingTargetStateSender>();
  builder.Connect(moving_target->get_output_port(0), state_sender->get_input_port(0));
  auto* state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<lcmt_moving_target_state>(
          params.lcm_channels.state, lcm, 1.0 / params.publish_rate));
  builder.Connect(state_sender->get_output_port(0), state_pub->get_input_port());

  if (params.console_log) {
    auto* logger =
        builder.AddSystem<MovingTargetConsoleLogger>(params.console_period);
    builder.Connect(moving_target->get_output_port(0), logger->get_input_port(0));
  }
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg);
  auto context = diagram->CreateDefaultContext();
  auto& moving_target_context =
      diagram->GetMutableSubsystemContext(*moving_target, context.get());
  moving_target_context.SetContinuousState(
      MakeMovingTargetInitialStateVector(params.initial_state));

  std::cout << "moving_target sim config: " << FLAGS_config << "\n";
  std::cout << "Model: " << params.model << "\n";
  std::cout << "Receiving actuation on " << params.lcm_channels.actuation_command
            << "\n";
  std::cout << "Publishing state on " << params.lcm_channels.state << "\n";

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
