#include <csignal>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/propeller.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "params/quadrotor_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_systems.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_quadrotor_command.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_sim_time.hpp"

DEFINE_string(config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_string(diagram_svg, "", "Path or directory for the system diagram SVG.");

namespace uav_delivery {
namespace {

class MultibodyQuadrotorState final : public drake::systems::LeafSystem<double> {
 public:
  MultibodyQuadrotorState() {
    state_port_ = this->DeclareVectorInputPort(
        "multibody_state", drake::systems::BasicVector<double>(13))
                      .get_index();
    this->DeclareVectorOutputPort("quadrotor_state",
                                  drake::systems::BasicVector<double>(18),
                                  &MultibodyQuadrotorState::CalcState);
  }

 private:
  void CalcState(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* output) const {
    const Eigen::VectorXd x = this->get_input_port(state_port_).Eval(context);
    const Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    Eigen::VectorXd state = Eigen::VectorXd::Zero(18);
    state.segment<3>(0) = x.segment<3>(4);
    state.segment<3>(3) = x.segment<3>(10);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(state.data() + 6) =
        R;
    state.segment<3>(15) = x.segment<3>(7);
    output->SetFromVector(state);
  }

  drake::systems::InputPortIndex state_port_;
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  QuadrotorSimParams params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);

  drake::systems::DiagramBuilder<double> builder;
  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);

  auto* plant = builder.AddSystem<drake::multibody::MultibodyPlant>(0.0);
  drake::multibody::Parser parser(plant);
  parser.package_map().Add("uav_models", "UAV_models");
  const auto models = parser.AddModels(params.model);
  const auto model_instance = models.at(0);
  plant->Finalize();
  const auto& base_link = plant->GetBodyByName("base_link", model_instance);

  auto* command_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_command>(
          params.lcm_channels.command, lcm));
  auto* command_receiver =
      builder.AddSystem<systems::QuadrotorCommandReceiver>();
  builder.Connect(command_sub->get_output_port(), command_receiver->get_input_port(0));

  const double rotor_offset = params.plant.arm_length / std::sqrt(2.0);
  std::vector<drake::multibody::PropellerInfo> propeller_info{
      drake::multibody::PropellerInfo(
          base_link.index(),
          drake::math::RigidTransform<double>(
              Eigen::Vector3d(rotor_offset, rotor_offset, 0.0)),
          params.plant.thrust_coeff, params.plant.yaw_moment_coeff),
      drake::multibody::PropellerInfo(
          base_link.index(),
          drake::math::RigidTransform<double>(
              Eigen::Vector3d(-rotor_offset, rotor_offset, 0.0)),
          params.plant.thrust_coeff, -params.plant.yaw_moment_coeff),
      drake::multibody::PropellerInfo(
          base_link.index(),
          drake::math::RigidTransform<double>(
              Eigen::Vector3d(-rotor_offset, -rotor_offset, 0.0)),
          params.plant.thrust_coeff, params.plant.yaw_moment_coeff),
      drake::multibody::PropellerInfo(
          base_link.index(),
          drake::math::RigidTransform<double>(
              Eigen::Vector3d(rotor_offset, -rotor_offset, 0.0)),
          params.plant.thrust_coeff, -params.plant.yaw_moment_coeff),
  };
  auto* propellers =
      builder.AddSystem<drake::multibody::Propeller<double>>(propeller_info);
  builder.Connect(command_receiver->get_output_port(0),
                  propellers->get_command_input_port());
  builder.Connect(plant->get_body_poses_output_port(),
                  propellers->get_body_poses_input_port());
  builder.Connect(propellers->get_spatial_forces_output_port(),
                  plant->get_applied_spatial_force_input_port());

  auto* state_sender = builder.AddSystem<systems::QuadrotorStateSender>();
  auto* state_converter = builder.AddSystem<MultibodyQuadrotorState>();
  builder.Connect(plant->get_state_output_port(model_instance),
                  state_converter->get_input_port(0));
  builder.Connect(state_converter->get_output_port(0),
                  state_sender->get_input_port(0));
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
        18, params.console_period);
    builder.Connect(state_converter->get_output_port(0),
                    console_logger->get_input_port(0));
  }
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg, argv[0]);
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant->GetMyMutableContextFromRoot(context.get());
  const drake::math::RigidTransform<double> X_WB(
      drake::math::RollPitchYaw<double>(params.initial_state.rpy),
      params.initial_state.position);
  plant->SetFreeBodyPose(&plant_context, base_link, X_WB);
  plant->SetFreeBodySpatialVelocity(
      &plant_context, base_link,
      drake::multibody::SpatialVelocity<double>(
          params.initial_state.body_angular_velocity,
          params.initial_state.velocity));

  std::cout << "Quadrotor sim config: " << FLAGS_config << "\n";
  std::cout << "Model: " << params.model << "\n";
  std::cout << "Using Drake MultibodyPlant + Propeller for rotor forces\n";
  std::cout << "Publishing state on " << params.lcm_channels.state
            << " and sim time on " << params.lcm_channels.sim_time << "\n";
  std::cout << "Receiving propeller inputs on " << params.lcm_channels.command
            << " as lcmt_quadrotor_command [u1,u2,u3,u4]\n";
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
