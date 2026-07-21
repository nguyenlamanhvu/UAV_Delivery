#include <cmath>
#include <csignal>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/value.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "params/quadrotor_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_systems.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_timestamped_saved_traj.hpp"

DEFINE_string(config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_int32(meshcat_port, 7000, "Port for Meshcat server.");
DEFINE_double(visualizer_publish_rate, 60.0, "Meshcat publish rate in Hz.");
DEFINE_string(diagram_svg, "", "Path or directory for the system diagram SVG.");

namespace uav_delivery {
namespace {

constexpr const char* kSe3ReferenceName = "quadrotor_se3_reference";
constexpr double kWaypointAxisLength = 0.18;

class QuadrotorStateToPosition final : public drake::systems::LeafSystem<double> {
 public:
  explicit QuadrotorStateToPosition(
      const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant),
        body_(plant.GetBodyByName("base_link")),
        plant_context_(plant.CreateDefaultContext()) {
    state_port_ = this->DeclareVectorInputPort(
        "quadrotor_state", drake::systems::BasicVector<double>(18))
                      .get_index();
    this->DeclareVectorOutputPort(
        "q", drake::systems::BasicVector<double>(plant.num_positions()),
        &QuadrotorStateToPosition::CalcPositions);
  }

 private:
  void CalcPositions(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* output) const {
    const Eigen::VectorXd state = this->get_input_port(state_port_).Eval(context);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(
        state.data() + 6);
    const drake::math::RigidTransform<double> X_WB(
        drake::math::RotationMatrix<double>(R), state.segment<3>(0));
    plant_.SetFreeBodyPose(plant_context_.get(), body_, X_WB);
    output->SetFromVector(plant_.GetPositions(*plant_context_));
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::RigidBody<double>& body_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  drake::systems::InputPortIndex state_port_;
};

class ReferenceTrajectoryVisualizer final
    : public drake::systems::LeafSystem<double> {
 public:
  ReferenceTrajectoryVisualizer(std::shared_ptr<drake::geometry::Meshcat> meshcat,
                                double publish_period)
      : meshcat_(std::move(meshcat)) {
    trajectory_port_ = this->DeclareAbstractInputPort(
                               "quadrotor_reference_trajectory",
                               drake::Value<lcmt_timestamped_saved_traj>{})
                           .get_index();
    this->DeclarePeriodicPublishEvent(
        publish_period, 0.0, &ReferenceTrajectoryVisualizer::DrawTrajectory);
  }

 private:
  drake::systems::EventStatus DrawTrajectory(
      const drake::systems::Context<double>& context) const {
    const auto& message =
        this->get_input_port(trajectory_port_)
            .Eval<lcmt_timestamped_saved_traj>(context);

    const lcmt_trajectory_block* block = nullptr;
    for (int i = 0; i < message.saved_traj.num_trajectories; ++i) {
      const auto& candidate = message.saved_traj.trajectories[i];
      const bool matching_block = candidate.trajectory_name == kSe3ReferenceName;
      const bool matching_name =
          i < static_cast<int>(message.saved_traj.trajectory_names.size()) &&
          message.saved_traj.trajectory_names[i] == kSe3ReferenceName;
      if (matching_block || matching_name) {
        block = &candidate;
        break;
      }
    }

    if (block == nullptr || block->num_points <= 0 ||
        block->num_datatypes < 3) {
      return drake::systems::EventStatus::Succeeded();
    }

    Eigen::Matrix3Xd vertices(3, block->num_points);
    for (int i = 0; i < block->num_points; ++i) {
      vertices.col(i) << block->datapoints[0][i], block->datapoints[1][i],
          block->datapoints[2][i];
    }
    meshcat_->SetLine("uav/reference_preview/path", vertices, 4.0,
                      drake::geometry::Rgba(0.95, 0.35, 0.05, 1.0));
    return drake::systems::EventStatus::Succeeded();
  }

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  drake::systems::InputPortIndex trajectory_port_;
};

void DrawConfiguredWaypoints(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const QuadrotorSimParams& params) {
  const auto& waypoints = params.trajectory.waypoints;
  if (waypoints.empty()) {
    return;
  }

  Eigen::Matrix3Xd axis_start(3, waypoints.size());
  Eigen::Matrix3Xd x_axis_end(3, waypoints.size());
  Eigen::Matrix3Xd y_axis_end(3, waypoints.size());
  Eigen::Matrix3Xd z_axis_end(3, waypoints.size());

  for (int i = 0; i < static_cast<int>(waypoints.size()); ++i) {
    const auto& waypoint = waypoints[i];
    const Eigen::Vector3d p = waypoint.position;
    const Eigen::Vector3d x_axis(std::cos(waypoint.yaw),
                                 std::sin(waypoint.yaw), 0.0);
    const Eigen::Vector3d y_axis(-std::sin(waypoint.yaw),
                                 std::cos(waypoint.yaw), 0.0);

    axis_start.col(i) = p;
    x_axis_end.col(i) = p + kWaypointAxisLength * x_axis;
    y_axis_end.col(i) = p + kWaypointAxisLength * y_axis;
    z_axis_end.col(i) = p + kWaypointAxisLength * Eigen::Vector3d::UnitZ();

    const std::string point_path =
        "uav/config_waypoints/point_" + std::to_string(i);
    meshcat->SetObject(point_path, drake::geometry::Sphere(0.035),
                       drake::geometry::Rgba(0.1, 0.45, 1.0, 1.0));
    meshcat->SetTransform(
        point_path,
        drake::math::RigidTransform<double>(
            drake::math::RotationMatrix<double>::Identity(), p));
  }

  meshcat->SetLineSegments("uav/config_waypoints/axis_x", axis_start,
                           x_axis_end, 3.0,
                           drake::geometry::Rgba(0.95, 0.05, 0.05, 1.0));
  meshcat->SetLineSegments("uav/config_waypoints/axis_y", axis_start,
                           y_axis_end, 3.0,
                           drake::geometry::Rgba(0.05, 0.75, 0.1, 1.0));
  meshcat->SetLineSegments("uav/config_waypoints/axis_z", axis_start,
                           z_axis_end, 3.0,
                           drake::geometry::Rgba(0.05, 0.25, 1.0, 1.0));
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  const QuadrotorSimParams params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);

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
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_state>(
          params.lcm_channels.state, lcm));
  auto* trajectory_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          lcmt_timestamped_saved_traj>(
          params.lcm_channels.reference_trajectory, lcm));
  auto* state_receiver = builder.AddSystem<systems::QuadrotorStateReceiver>();
  auto* state_to_q = builder.AddSystem<QuadrotorStateToPosition>(plant);
  auto* to_pose = builder.AddSystem<
      drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(plant);

  builder.Connect(state_sub->get_output_port(), state_receiver->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0), state_to_q->get_input_port(0));
  builder.Connect(state_to_q->get_output_port(0), to_pose->get_input_port());
  builder.Connect(to_pose->get_output_port(),
                  scene_graph->get_source_pose_port(plant.get_source_id().value()));

  auto meshcat = std::make_shared<drake::geometry::Meshcat>(FLAGS_meshcat_port);
  drake::geometry::MeshcatVisualizerParams meshcat_params;
  meshcat_params.publish_period = 1.0 / FLAGS_visualizer_publish_rate;
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, *scene_graph, meshcat, std::move(meshcat_params));
  auto* reference_viz = builder.AddSystem<ReferenceTrajectoryVisualizer>(
      meshcat, 1.0 / FLAGS_visualizer_publish_rate);
  builder.Connect(trajectory_sub->get_output_port(),
                  reference_viz->get_input_port(0));
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph);
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg, argv[0]);
  auto context = diagram->CreateDefaultContext();
  DrawConfiguredWaypoints(meshcat, params);

  std::cout << "Quadrotor visualizer config: " << FLAGS_config << "\n";
  std::cout << "Model: " << params.model << "\n";
  std::cout << "Subscribing state on " << params.lcm_channels.state << "\n";
  std::cout << "Subscribing trajectory preview on "
            << params.lcm_channels.reference_trajectory << "\n";
  std::cout << "Showing configured waypoint yaw-aligned XYZ markers from trajectory.waypoints\n";
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
