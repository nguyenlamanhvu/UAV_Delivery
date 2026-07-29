#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include "params/quadrotor_camera_visualizer_params.h"
#include "uav_delivery/lcmt_raruco_detection.hpp"

namespace uav_delivery {
namespace systems {

// Shared-world RArUco detector for the quadrotor front camera. It uses the
// known quadrotor + moving-target states to seed the marker projection, then
// verifies the projected patch against the expected recursive marker image.
// This gives the camera pipeline a real image-based marker confirmation path
// without introducing an external OpenCV / aruco2 dependency into the repo.
class ProjectedRArucoDetector final : public drake::systems::LeafSystem<double> {
 public:
  ProjectedRArucoDetector(const DroneCameraRenderParams& camera_params,
                          const RArucoDetectorParams& detector_params);

 private:
  void CalcDetection(const drake::systems::Context<double>& context,
                     lcmt_raruco_detection* output) const;
  void CalcAnnotatedImage(
      const drake::systems::Context<double>& context,
      drake::systems::sensors::ImageRgba8U* output) const;

  drake::systems::InputPortIndex quadrotor_state_input_port_;
  drake::systems::InputPortIndex moving_target_state_input_port_;
  drake::systems::InputPortIndex color_image_input_port_;

  DroneCameraRenderParams camera_params_;
  RArucoDetectorParams detector_params_;
  drake::math::RigidTransformd X_BC_;
  std::vector<uint8_t> expected_marker_;
};

}  // namespace systems
}  // namespace uav_delivery
