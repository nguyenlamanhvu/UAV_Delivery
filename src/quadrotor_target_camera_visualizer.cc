#include <algorithm>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
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
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "params/moving_target_params.h"
#include "params/quadrotor_camera_visualizer_params.h"
#include "params/quadrotor_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_systems.h"
#include "systems/moving_target_lcm_systems.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_moving_target_state.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"

DEFINE_string(quadrotor_config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(moving_target_config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams.");
DEFINE_string(camera_config, "config/quadrotor_target_camera_visualizer.yaml",
              "YAML file containing QuadrotorTargetCameraVisualizerParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_string(diagram_svg, "", "Optional path to write the system diagram SVG.");

namespace uav_delivery {
namespace {

enum class EngineType { kVtk, kGl };

class CombinedSceneStateToPosition final
    : public drake::systems::LeafSystem<double> {
 public:
  CombinedSceneStateToPosition(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex quadrotor_instance,
      drake::multibody::ModelInstanceIndex moving_target_instance)
      : plant_(plant),
        quadrotor_body_(plant.GetBodyByName("base_link", quadrotor_instance)),
        moving_target_body_(plant.GetBodyByName("base_link", moving_target_instance)),
        front_left_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "front_left_wheel_joint", moving_target_instance)),
        front_right_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "front_right_wheel_joint", moving_target_instance)),
        rear_left_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "rear_left_wheel_joint", moving_target_instance)),
        rear_right_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "rear_right_wheel_joint", moving_target_instance)),
        plant_context_(plant.CreateDefaultContext()) {
    quadrotor_state_port_ = this->DeclareVectorInputPort(
                                "quadrotor_state",
                                drake::systems::BasicVector<double>(18))
                                .get_index();
    moving_target_state_port_ = this->DeclareVectorInputPort(
                                    "moving_target_state",
                                    drake::systems::BasicVector<double>(9))
                                    .get_index();
    this->DeclareVectorOutputPort(
        "q", drake::systems::BasicVector<double>(plant.num_positions()),
        &CombinedSceneStateToPosition::CalcPositions);
  }

 private:
  void CalcPositions(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* output) const {
    const Eigen::VectorXd quadrotor_state =
        this->get_input_port(quadrotor_state_port_).Eval(context);
    const Eigen::VectorXd moving_target_state =
        this->get_input_port(moving_target_state_port_).Eval(context);

    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_WQ(
        quadrotor_state.data() + 6);
    const drake::math::RigidTransform<double> X_WQ(
        drake::math::RotationMatrix<double>(R_WQ), quadrotor_state.segment<3>(0));
    plant_.SetFreeBodyPose(plant_context_.get(), quadrotor_body_, X_WQ);

    const drake::math::RigidTransform<double> X_WT(
        drake::math::RollPitchYaw<double>(0.0, 0.0, moving_target_state(2)),
        Eigen::Vector3d(moving_target_state(0), moving_target_state(1), 0.0));
    plant_.SetFreeBodyPose(plant_context_.get(), moving_target_body_, X_WT);
    front_left_.set_angle(plant_context_.get(), moving_target_state(5));
    front_right_.set_angle(plant_context_.get(), moving_target_state(6));
    rear_left_.set_angle(plant_context_.get(), moving_target_state(7));
    rear_right_.set_angle(plant_context_.get(), moving_target_state(8));

    output->SetFromVector(plant_.GetPositions(*plant_context_));
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::RigidBody<double>& quadrotor_body_;
  const drake::multibody::RigidBody<double>& moving_target_body_;
  const drake::multibody::RevoluteJoint<double>& front_left_;
  const drake::multibody::RevoluteJoint<double>& front_right_;
  const drake::multibody::RevoluteJoint<double>& rear_left_;
  const drake::multibody::RevoluteJoint<double>& rear_right_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  drake::systems::InputPortIndex quadrotor_state_port_;
  drake::systems::InputPortIndex moving_target_state_port_;
};

template <EngineType engine_type>
std::unique_ptr<drake::geometry::render::RenderEngine> MakeEngine() {
  if constexpr (engine_type == EngineType::kVtk) {
    return drake::geometry::MakeRenderEngineVtk(
        drake::geometry::RenderEngineVtkParams{});
  }
  return drake::geometry::MakeRenderEngineGl(
      drake::geometry::RenderEngineGlParams{});
}

EngineType ParseRenderer(const std::string& renderer) {
  if (renderer == "vtk") {
    return EngineType::kVtk;
  }
  if (renderer == "gl") {
    return EngineType::kGl;
  }
  throw std::runtime_error("Unsupported renderer='" + renderer +
                           "'. Expected 'vtk' or 'gl'.");
}

drake::math::RigidTransformd MakeDroneCameraPose(
    const DroneCameraRenderParams& params) {
  const double pitch_rad = params.pitch_down_deg * M_PI / 180.0;
  const Eigen::Vector3d look_direction(std::cos(pitch_rad), 0.0,
                                       -std::sin(pitch_rad));
  const Eigen::Vector3d down_reference(0.0, 0.0, -1.0);
  Eigen::Vector3d camera_y =
      down_reference - down_reference.dot(look_direction) * look_direction;
  if (camera_y.norm() < 1e-8) {
    camera_y = Eigen::Vector3d(0.0, -1.0, 0.0);
  }
  camera_y.normalize();
  const Eigen::Vector3d camera_z = look_direction.normalized();
  const Eigen::Vector3d camera_x = camera_y.cross(camera_z).normalized();

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix.col(0) = camera_x;
  rotation_matrix.col(1) = camera_y;
  rotation_matrix.col(2) = camera_z;

  return drake::math::RigidTransformd(
      drake::math::RotationMatrix<double>(rotation_matrix), params.position);
}

std::string CameraOutputFormat(
    const DroneCameraImageOutputParams& image_output_params) {
  return image_output_params.output_dir + "/" + image_output_params.file_pattern;
}

void WarmStartCamera(drake::systems::Diagram<double>* diagram,
                     drake::systems::Context<double>* context,
                     const drake::systems::sensors::RgbdSensor<double>* camera,
                     int warmup_frames) {
  std::cout << "Warming up drone camera rendering pipeline..." << std::endl;
  for (int frame = 0; frame < warmup_frames; ++frame) {
    const auto& camera_context = diagram->GetSubsystemContext(*camera, *context);
    const auto& color_image =
        camera->color_image_output_port()
            .Eval<drake::systems::sensors::ImageRgba8U>(camera_context);
    std::cout << "  warmup frame " << (frame + 1) << "/" << warmup_frames
              << " -> " << color_image.width() << "x" << color_image.height()
              << std::endl;
  }
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::signal(SIGINT, systems::HandleSigint);

  const QuadrotorSimParams quadrotor_params =
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_quadrotor_config);
  const MovingTargetSimParams moving_target_params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_moving_target_config);
  const QuadrotorTargetCameraVisualizerParams camera_visualizer_params =
      drake::yaml::LoadYamlFile<QuadrotorTargetCameraVisualizerParams>(
          FLAGS_camera_config);

  std::filesystem::create_directories(
      camera_visualizer_params.image_output.output_dir);

  drake::systems::DiagramBuilder<double> builder;
  drake::multibody::MultibodyPlant<double> plant(0.0);
  auto* scene_graph = builder.AddSystem<drake::geometry::SceneGraph>();
  scene_graph->set_name("scene_graph");

  drake::multibody::Parser parser(&plant, scene_graph);
  parser.package_map().Add("uav_models", "UAV_models");

  const auto quadrotor_instances = parser.AddModels(quadrotor_params.model);
  const auto moving_target_instances = parser.AddModels(moving_target_params.model);
  if (quadrotor_instances.size() != 1 || moving_target_instances.size() != 1) {
    throw std::runtime_error(
        "Expected exactly one model instance for the quadrotor and moving target.");
  }
  const auto quadrotor_instance = quadrotor_instances.front();
  const auto moving_target_instance = moving_target_instances.front();
  plant.Finalize();

  auto* lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto* quadrotor_state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_state>(
          quadrotor_params.lcm_channels.state, lcm));
  auto* quadrotor_state_receiver =
      builder.AddSystem<systems::QuadrotorStateReceiver>();
  auto* moving_target_state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_moving_target_state>(
          moving_target_params.lcm_channels.state, lcm));
  auto* moving_target_state_receiver =
      builder.AddSystem<systems::MovingTargetStateReceiver>();
  auto* state_to_q = builder.AddSystem<CombinedSceneStateToPosition>(
      plant, quadrotor_instance, moving_target_instance);
  auto* to_pose = builder.AddSystem<
      drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(plant);

  builder.Connect(quadrotor_state_sub->get_output_port(),
                  quadrotor_state_receiver->get_input_port(0));
  builder.Connect(quadrotor_state_receiver->get_output_port(0),
                  state_to_q->get_input_port(0));
  builder.Connect(moving_target_state_sub->get_output_port(),
                  moving_target_state_receiver->get_input_port(0));
  builder.Connect(moving_target_state_receiver->get_output_port(0),
                  state_to_q->get_input_port(1));
  builder.Connect(state_to_q->get_output_port(0), to_pose->get_input_port());
  builder.Connect(to_pose->get_output_port(),
                  scene_graph->get_source_pose_port(plant.get_source_id().value()));

  const std::string renderer_name = "drone_front_renderer";
  switch (ParseRenderer(camera_visualizer_params.renderer)) {
    case EngineType::kVtk:
      scene_graph->AddRenderer(renderer_name, MakeEngine<EngineType::kVtk>());
      break;
    case EngineType::kGl:
      scene_graph->AddRenderer(renderer_name, MakeEngine<EngineType::kGl>());
      break;
  }

  const auto& camera_params = camera_visualizer_params.camera;
  const drake::systems::sensors::CameraInfo camera_info(
      camera_params.width, camera_params.height,
      camera_params.fov_deg * M_PI / 180.0);
  const drake::geometry::render::RenderCameraCore color_camera_core(
      renderer_name, camera_info,
      drake::geometry::render::ClippingRange{camera_params.near, camera_params.far},
      drake::math::RigidTransformd::Identity());
  const drake::geometry::render::ColorRenderCamera color_camera(
      color_camera_core, false);
  const drake::geometry::render::DepthRenderCamera depth_camera(
      color_camera_core,
      drake::geometry::render::DepthRange(camera_params.near, camera_params.far));
  const drake::math::RigidTransformd X_BC = MakeDroneCameraPose(camera_params);

  auto* drone_camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
      plant.GetBodyFrameIdOrThrow(
          plant.GetBodyByName("base_link", quadrotor_instance).index()),
      X_BC, color_camera, depth_camera);
  builder.Connect(scene_graph->get_query_output_port(),
                  drone_camera->query_object_input_port());

  auto* image_writer = builder.AddSystem<drake::systems::sensors::ImageWriter>();
  const auto& image_input = image_writer->DeclareImageInputPort(
      drake::systems::sensors::PixelType::kRgba8U, "drone_front_camera",
      CameraOutputFormat(camera_visualizer_params.image_output),
      camera_visualizer_params.image_output.publish_period, 0.0);
  builder.Connect(drone_camera->color_image_output_port(), image_input);

  auto meshcat = std::make_shared<drake::geometry::Meshcat>(
      camera_visualizer_params.meshcat_port);
  drake::geometry::MeshcatVisualizerParams meshcat_params;
  meshcat_params.publish_period =
      1.0 / camera_visualizer_params.visualizer_publish_rate;
  meshcat_params.prefix = "/combined_scene";
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, *scene_graph, meshcat, std::move(meshcat_params));
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph);
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg);
  auto context = diagram->CreateDefaultContext();

  std::cout << "Quadrotor camera visualizer" << std::endl;
  std::cout << "  quadrotor config: " << FLAGS_quadrotor_config << std::endl;
  std::cout << "  moving target config: " << FLAGS_moving_target_config
            << std::endl;
  std::cout << "  camera config: " << FLAGS_camera_config << std::endl;
  std::cout << "  quadrotor state channel: " << quadrotor_params.lcm_channels.state
            << std::endl;
  std::cout << "  moving target state channel: "
            << moving_target_params.lcm_channels.state << std::endl;
  std::cout << "  camera pose X_BC:\n" << X_BC.GetAsMatrix4() << std::endl;
  std::cout << "  saving camera frames to: "
            << CameraOutputFormat(camera_visualizer_params.image_output)
            << std::endl;
  std::cout << "  Meshcat: " << meshcat->web_url() << std::endl;

  WarmStartCamera(diagram.get(), context.get(), drone_camera,
                  camera_params.warmup_frames);

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(
      std::max(quadrotor_params.realtime_rate, moving_target_params.realtime_rate));
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace
}  // namespace uav_delivery

int main(int argc, char* argv[]) {
  return uav_delivery::DoMain(argc, argv);
}
