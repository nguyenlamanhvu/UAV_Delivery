#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

namespace uav_delivery {

struct DroneCameraRenderParams {
  int width{640};
  int height{480};
  double fov_deg{75.0};
  double near{0.02};
  double far{50.0};
  Eigen::Vector3d position{0.18, 0.0, -0.03};
  double pitch_down_deg{25.0};
  int warmup_frames{3};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(width));
    a->Visit(DRAKE_NVP(height));
    a->Visit(DRAKE_NVP(fov_deg));
    a->Visit(DRAKE_NVP(near));
    a->Visit(DRAKE_NVP(far));
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(pitch_down_deg));
    a->Visit(DRAKE_NVP(warmup_frames));
  }
};

struct DroneCameraImageOutputParams {
  std::string output_dir{"/tmp/uav_delivery/drone_front_camera"};
  std::string file_pattern{"drone_front_{count:03}"};
  double publish_period{0.5};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(output_dir));
    a->Visit(DRAKE_NVP(file_pattern));
    a->Visit(DRAKE_NVP(publish_period));
  }
};

struct DroneCameraAnnotatedOutputParams {
  bool enabled{true};
  std::string output_dir{"/tmp/uav_delivery/drone_front_camera_raruco"};
  std::string file_pattern{"drone_front_raruco_{count:03}"};
  double publish_period{0.5};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(enabled));
    a->Visit(DRAKE_NVP(output_dir));
    a->Visit(DRAKE_NVP(file_pattern));
    a->Visit(DRAKE_NVP(publish_period));
  }
};

struct RArucoMarkerParams {
  int id{0};
  int depth{2};
  int inner_borders{2};
  bool external_border{true};
  double size_m{0.4};
  Eigen::Vector3d position{0.0, 0.0, 0.221};
  Eigen::Vector3d rpy{0.0, 0.0, 0.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(id));
    a->Visit(DRAKE_NVP(depth));
    a->Visit(DRAKE_NVP(inner_borders));
    a->Visit(DRAKE_NVP(external_border));
    a->Visit(DRAKE_NVP(size_m));
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(rpy));
  }
};

struct RArucoDetectorParams {
  bool enabled{true};
  std::string lcm_channel{"UAV_RARUCO_DETECTION"};
  double publish_period{0.5};
  int warp_resolution{240};
  double min_marker_area_px{400.0};
  double min_normal_alignment{0.25};
  double min_in_frame_fraction{0.7};
  double max_mean_abs_error{70.0};
  DroneCameraAnnotatedOutputParams overlay_output;
  RArucoMarkerParams marker;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(enabled));
    a->Visit(DRAKE_NVP(lcm_channel));
    a->Visit(DRAKE_NVP(publish_period));
    a->Visit(DRAKE_NVP(warp_resolution));
    a->Visit(DRAKE_NVP(min_marker_area_px));
    a->Visit(DRAKE_NVP(min_normal_alignment));
    a->Visit(DRAKE_NVP(min_in_frame_fraction));
    a->Visit(DRAKE_NVP(max_mean_abs_error));
    a->Visit(DRAKE_NVP(overlay_output));
    a->Visit(DRAKE_NVP(marker));
  }
};

struct QuadrotorTargetCameraVisualizerParams {
  std::string renderer{"vtk"};
  int meshcat_port{7002};
  double visualizer_publish_rate{60.0};
  DroneCameraRenderParams camera;
  DroneCameraImageOutputParams image_output;
  RArucoDetectorParams detection;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(renderer));
    a->Visit(DRAKE_NVP(meshcat_port));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(camera));
    a->Visit(DRAKE_NVP(image_output));
    a->Visit(DRAKE_NVP(detection));
  }
};

}  // namespace uav_delivery
