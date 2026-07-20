#include <csignal>
#include <limits>
#include <memory>
#include <utility>
#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "params/moving_target_params.h"
#include "systems/diagram_utils.h"
#include "systems/moving_target_lcm_systems.h"
#include "systems/moving_target_plant.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_moving_target_state.hpp"

DEFINE_string(config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_int32(meshcat_port, 7001, "Port for Meshcat server.");
DEFINE_double(visualizer_publish_rate, 60.0, "Meshcat publish rate in Hz.");
DEFINE_string(diagram_svg, "", "Optional path to write the system diagram SVG.");

namespace uav_delivery {
namespace {

class MovingTargetStateToPosition final : public drake::systems::LeafSystem<double> {
 public:
  explicit MovingTargetStateToPosition(
      const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant),
        body_(plant.GetBodyByName("base_link")),
        front_left_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "front_left_wheel_joint")),
        front_right_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "front_right_wheel_joint")),
        rear_left_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "rear_left_wheel_joint")),
        rear_right_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "rear_right_wheel_joint")),
        plant_context_(plant.CreateDefaultContext()) {
    state_port_ = this->DeclareVectorInputPort(
        "moving_target_state",
        drake::systems::BasicVector<double>(systems::MovingTargetPlant::kStateSize))
                      .get_index();
    this->DeclareVectorOutputPort(
        "q", drake::systems::BasicVector<double>(plant.num_positions()),
        &MovingTargetStateToPosition::CalcPositions);
  }

 private:
  void CalcPositions(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* output) const {
    const Eigen::VectorXd state = this->get_input_port(state_port_).Eval(context);
    const drake::math::RigidTransform<double> X_WB(
        drake::math::RollPitchYaw<double>(0.0, 0.0, state(2)),
        Eigen::Vector3d(state(0), state(1), 0.0));
    plant_.SetFreeBodyPose(plant_context_.get(), body_, X_WB);
    front_left_.set_angle(plant_context_.get(), state(5));
    front_right_.set_angle(plant_context_.get(), state(6));
    rear_left_.set_angle(plant_context_.get(), state(7));
    rear_right_.set_angle(plant_context_.get(), state(8));
    output->SetFromVector(plant_.GetPositions(*plant_context_));
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::RigidBody<double>& body_;
  const drake::multibody::RevoluteJoint<double>& front_left_;
  const drake::multibody::RevoluteJoint<double>& front_right_;
  const drake::multibody::RevoluteJoint<double>& rear_left_;
  const drake::multibody::RevoluteJoint<double>& rear_right_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  drake::systems::InputPortIndex state_port_;
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  const MovingTargetSimParams params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_config);

  drake::systems::DiagramBuilder<double> builder;
  drake::multibody::MultibodyPlant<double> plant(0.0);
  auto* scene_graph = builder.AddSystem<drake::geometry::SceneGraph>();
  scene_graph->set_name("scene_graph");

  drake::multibody::Parser parser(&plant, scene_graph);
  parser.package_map().Add("uav_models", "UAV_models");
  parser.AddModels(params.model);
  plant.Finalize();

  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto* state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_moving_target_state>(
          params.lcm_channels.state, lcm));
  auto* state_receiver = builder.AddSystem<systems::MovingTargetStateReceiver>();
  auto* state_to_q = builder.AddSystem<MovingTargetStateToPosition>(plant);
  auto* to_pose = builder.AddSystem<
      drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(plant);

  builder.Connect(state_sub->get_output_port(), state_receiver->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0), state_to_q->get_input_port(0));
  builder.Connect(state_to_q->get_output_port(0), to_pose->get_input_port());
  builder.Connect(to_pose->get_output_port(),
                  scene_graph->get_source_pose_port(plant.get_source_id().value()));

  auto meshcat = std::make_shared<drake::geometry::Meshcat>(FLAGS_meshcat_port);
  const std::string meshcat_root = params.actor.meshcat_root.empty()
                                       ? systems::ActorMeshcatPath(params.actor.actor_name)
                                       : params.actor.meshcat_root;
  drake::geometry::MeshcatVisualizerParams meshcat_params;
  meshcat_params.publish_period = 1.0 / FLAGS_visualizer_publish_rate;
  meshcat_params.prefix = meshcat_root;
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, *scene_graph, meshcat, std::move(meshcat_params));
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph);
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg);
  auto context = diagram->CreateDefaultContext();

  std::cout << "moving_target visualizer config: " << FLAGS_config << "\n";
  std::cout << "Model: " << params.model << "\n";
  std::cout << "Subscribing state on " << params.lcm_channels.state << "\n";
  std::cout << "Meshcat root: " << meshcat_root << "\n";
  std::cout << "Meshcat: " << meshcat->web_url() << std::endl;

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(params.realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace
}  // namespace uav_delivery

int main(int argc, char* argv[]) {
  return uav_delivery::DoMain(argc, argv);
}
