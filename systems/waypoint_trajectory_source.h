#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"
#include "params/quadrotor_params.h"
#include "uav_delivery/lcmt_quadrotor_reference.hpp"
#include "uav_delivery/lcmt_quadrotor_state.hpp"

namespace uav_delivery {
namespace systems {

class WaypointTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  explicit WaypointTrajectorySource(QuadrotorTrajectoryParams params);

 private:
  struct SegmentReference {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    double yaw{};
    double yaw_rate{};
    double yaw_acceleration{};
  };

  drake::systems::EventStatus UpdateReference(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;
  void CalcReference(const drake::systems::Context<double>& context,
                     lcmt_quadrotor_reference* output) const;
  lcmt_quadrotor_reference MakeReference(double time) const;
  SegmentReference Evaluate(double time) const;

  QuadrotorTrajectoryParams params_;
  std::vector<QuadrotorWaypoint> waypoints_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::AbstractStateIndex reference_state_index_;
};

}  // namespace systems
}  // namespace uav_delivery
