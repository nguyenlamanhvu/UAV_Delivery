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

struct QuadrotorTargetCameraVisualizerParams {
  std::string renderer{"vtk"};
  int meshcat_port{7002};
  double visualizer_publish_rate{60.0};
  DroneCameraRenderParams camera;
  DroneCameraImageOutputParams image_output;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(renderer));
    a->Visit(DRAKE_NVP(meshcat_port));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(camera));
    a->Visit(DRAKE_NVP(image_output));
  }
};

}  // namespace uav_delivery
