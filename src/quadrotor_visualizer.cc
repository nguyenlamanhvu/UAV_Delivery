#include <algorithm>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
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
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "systems/uav_image_system.h"
#include "params/moving_target_params.h"
#include "params/quadrotor_camera_visualizer_params.h"
#include "params/quadrotor_params.h"
#include "systems/diagram_utils.h"
#include "systems/lcm_systems.h"
#include "systems/moving_target_lcm_systems.h"
#include "systems/moving_target_plant.h"
#include "systems/raruco_detector.h"
#include "systems/sim_utils.h"
#include "uav_delivery/lcmt_moving_target_state.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_raruco_detection.hpp"

DEFINE_string(config, "config/quadrotor_sim.yaml",
              "YAML file containing QuadrotorSimParams.");
DEFINE_string(moving_target_config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams. The moving target is "
              "loaded into the same Meshcat scene as the quadrotor.");
DEFINE_string(camera_config, "config/quadrotor_target_camera_visualizer.yaml",
              "YAML file containing QuadrotorTargetCameraVisualizerParams.");
DEFINE_bool(camera_render, false,
            "Enable the onboard drone camera rendering and RArUco pipeline.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");
DEFINE_int32(meshcat_port, 7000, "Port for Meshcat server when camera_render=false.");
DEFINE_double(visualizer_publish_rate, 60.0,
              "Meshcat publish rate in Hz when camera_render=false.");
DEFINE_string(diagram_svg, "", "Path or directory for the system diagram SVG.");

namespace uav_delivery {
namespace {

enum class EngineType { kVtk, kGl };

class CombinedSceneStateToPosition final
    : public drake::systems::LeafSystem<double> {
 public:
  CombinedSceneStateToPosition(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex quadrotor_instance,
      drake::multibody::ModelInstanceIndex moving_target_instance,
      double camera_pitch_rad)
      : plant_(plant),
        quadrotor_body_(plant.GetBodyByName("base_link", quadrotor_instance)),
        moving_target_body_(plant.GetBodyByName("base_link", moving_target_instance)),
        camera_pitch_joint_(plant.GetJointByName<drake::multibody::RevoluteJoint>(
            "camera_pitch_joint", quadrotor_instance)),
        camera_pitch_rad_(camera_pitch_rad),
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
                                    drake::systems::BasicVector<double>(
                                        systems::MovingTargetPlant::kStateSize))
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
    camera_pitch_joint_.set_angle(plant_context_.get(), camera_pitch_rad_);

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
  const drake::multibody::RevoluteJoint<double>& camera_pitch_joint_;
  const double camera_pitch_rad_;
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

std::string CameraOutputFormat(
    const DroneCameraImageOutputParams& image_output_params) {
  return image_output_params.output_dir + "/" + image_output_params.file_pattern;
}

std::string OverlayOutputFormat(
    const DroneCameraAnnotatedOutputParams& overlay_output_params) {
  return overlay_output_params.output_dir + "/" + overlay_output_params.file_pattern;
}

void WarmStartCamera(drake::systems::Diagram<double>* diagram,
                     drake::systems::Context<double>* context,
                     const drake::systems::sensors::RgbdSensor* camera,
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
      drake::yaml::LoadYamlFile<QuadrotorSimParams>(FLAGS_config);
  const MovingTargetSimParams moving_target_params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_moving_target_config);

  std::optional<QuadrotorTargetCameraVisualizerParams> camera_visualizer_params;
  if (FLAGS_camera_render) {
    camera_visualizer_params =
        drake::yaml::LoadYamlFile<QuadrotorTargetCameraVisualizerParams>(
            FLAGS_camera_config);
    std::filesystem::create_directories(
        camera_visualizer_params->image_output.output_dir);
    if (camera_visualizer_params->detection.enabled &&
        camera_visualizer_params->detection.overlay_output.enabled) {
      std::filesystem::create_directories(
          camera_visualizer_params->detection.overlay_output.output_dir);
    }
  }

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
      plant, quadrotor_instance, moving_target_instance,
      camera_visualizer_params.has_value()
          ? camera_visualizer_params->camera.pitch_down_deg * M_PI / 180.0
          : 0.0);
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

  drake::systems::sensors::RgbdSensor* drone_camera = nullptr;
  if (camera_visualizer_params.has_value()) {
    const std::string renderer_name = "drone_front_renderer";
    switch (ParseRenderer(camera_visualizer_params->renderer)) {
      case EngineType::kVtk:
        scene_graph->AddRenderer(renderer_name, MakeEngine<EngineType::kVtk>());
        break;
      case EngineType::kGl:
        scene_graph->AddRenderer(renderer_name, MakeEngine<EngineType::kGl>());
        break;
    }

    const auto& camera_params = camera_visualizer_params->camera;
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
    drone_camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
        plant.GetBodyFrameIdOrThrow(
            plant.GetBodyByName("camera_link", quadrotor_instance).index()),
        drake::math::RigidTransformd::Identity(), color_camera, depth_camera);
    builder.Connect(scene_graph->get_query_output_port(),
                    drone_camera->query_object_input_port());

    int fps = 30; // 30 Hz default for gstreamer
    if (camera_visualizer_params->image_output.publish_period > 0.0) {
      fps = std::max(1, static_cast<int>(1.0 / camera_visualizer_params->image_output.publish_period));
    }
    auto* uav_image_system = builder.AddSystem<UavImageSystem>(
        fps, camera_params.width, camera_params.height,
        drake::systems::sensors::ImageRgba8U::kNumChannels, 1,
        "127.0.0.1", "8554", false);
    builder.Connect(drone_camera->color_image_output_port(),
                    uav_image_system->get_input_port_image_1());

    if (camera_visualizer_params->detection.enabled) {
      auto* raruco_detector = builder.AddSystem<systems::ProjectedRArucoDetector>(
          camera_visualizer_params->camera,
          camera_visualizer_params->detection);
      builder.Connect(quadrotor_state_receiver->get_output_port(0),
                      raruco_detector->get_input_port(0));
      builder.Connect(moving_target_state_receiver->get_output_port(0),
                      raruco_detector->get_input_port(1));
      builder.Connect(drone_camera->color_image_output_port(),
                      raruco_detector->get_input_port(2));

      auto* detection_pub = builder.AddSystem(
          drake::systems::lcm::LcmPublisherSystem::Make<lcmt_raruco_detection>(
              camera_visualizer_params->detection.lcm_channel, lcm,
              camera_visualizer_params->detection.publish_period));
      builder.Connect(raruco_detector->get_output_port(0),
                      detection_pub->get_input_port());

      if (camera_visualizer_params->detection.overlay_output.enabled) {
        auto* overlay_writer =
            builder.AddSystem<drake::systems::sensors::ImageWriter>();
        const auto& overlay_input = overlay_writer->DeclareImageInputPort(
            drake::systems::sensors::PixelType::kRgba8U,
            "drone_front_camera_raruco_overlay",
            OverlayOutputFormat(camera_visualizer_params->detection.overlay_output),
            camera_visualizer_params->detection.overlay_output.publish_period, 0.0);
      builder.Connect(raruco_detector->get_output_port(1), overlay_input);
      }
    }
  }

  const int meshcat_port = camera_visualizer_params.has_value()
                               ? camera_visualizer_params->meshcat_port
                               : FLAGS_meshcat_port;
  const double visualizer_publish_rate = camera_visualizer_params.has_value()
                                             ? camera_visualizer_params
                                                   ->visualizer_publish_rate
                                             : FLAGS_visualizer_publish_rate;

  auto meshcat = std::make_shared<drake::geometry::Meshcat>(meshcat_port);
  drake::geometry::MeshcatVisualizerParams meshcat_params;
  meshcat_params.publish_period = 1.0 / visualizer_publish_rate;
  if (camera_visualizer_params.has_value()) {
    meshcat_params.prefix = "/combined_scene";
  }
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, *scene_graph, meshcat, std::move(meshcat_params));
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph);
  builder.AddSystem<systems::SimTerminator>();

  auto diagram = builder.Build();
  systems::MaybeWriteDiagramSvg(*diagram, FLAGS_diagram_svg, argv[0]);
  auto context = diagram->CreateDefaultContext();

  std::cout << "Quadrotor visualizer" << std::endl;
  std::cout << "  quadrotor config: " << FLAGS_config << std::endl;
  std::cout << "  moving target config: " << FLAGS_moving_target_config
            << std::endl;
  std::cout << "  camera_render: " << (FLAGS_camera_render ? "true" : "false")
            << std::endl;
  if (camera_visualizer_params.has_value()) {
    std::cout << "  camera config: " << FLAGS_camera_config << std::endl;
    std::cout << "  camera body link: camera_link" << std::endl;
    std::cout << "  camera pitch joint angle [deg]: "
              << camera_visualizer_params->camera.pitch_down_deg << std::endl;
    std::cout << "  saving camera frames to: "
              << CameraOutputFormat(camera_visualizer_params->image_output)
              << std::endl;
    if (camera_visualizer_params->detection.enabled) {
      std::cout << "  RArUco marker id: "
                << camera_visualizer_params->detection.marker.id << std::endl;
      std::cout << "  RArUco channel: "
                << camera_visualizer_params->detection.lcm_channel << std::endl;
      if (camera_visualizer_params->detection.overlay_output.enabled) {
        std::cout << "  saving RArUco overlay frames to: "
                  << OverlayOutputFormat(
                         camera_visualizer_params->detection.overlay_output)
                  << std::endl;
      }
    }
  }
  std::cout << "  quadrotor state channel: "
            << quadrotor_params.lcm_channels.state << std::endl;
  std::cout << "  moving target state channel: "
            << moving_target_params.lcm_channels.state << std::endl;
  std::cout << "  Meshcat: " << meshcat->web_url() << std::endl;

  if (camera_visualizer_params.has_value()) {
    WarmStartCamera(diagram.get(), context.get(), drone_camera,
                    camera_visualizer_params->camera.warmup_frames);
  }

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
