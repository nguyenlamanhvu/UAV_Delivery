#pragma once

#include <array>
#include <string>

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

namespace uav_delivery {

struct MovingTargetPlantParams {
  double mass{18.0};
  double yaw_inertia{3.2};
  double wheel_radius{0.12};
  double track_width{0.5};
  double wheel_base{0.6};
  double k_drive{0.32};
  double k_turn{0.42};
  double d_v{0.35};
  double d_yaw{0.55};
  double k_wheel{2.8};
  double max_drive_torque{12.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mass));
    a->Visit(DRAKE_NVP(yaw_inertia));
    a->Visit(DRAKE_NVP(wheel_radius));
    a->Visit(DRAKE_NVP(track_width));
    a->Visit(DRAKE_NVP(wheel_base));
    a->Visit(DRAKE_NVP(k_drive));
    a->Visit(DRAKE_NVP(k_turn));
    a->Visit(DRAKE_NVP(d_v));
    a->Visit(DRAKE_NVP(d_yaw));
    a->Visit(DRAKE_NVP(k_wheel));
    a->Visit(DRAKE_NVP(max_drive_torque));
  }
};

struct MovingTargetInitialState {
  Eigen::Vector2d position{0.0, 0.0};
  double yaw{0.0};
  double forward_speed{0.0};
  double yaw_rate{0.0};
  std::array<double, 4> wheel_angles{0.0, 0.0, 0.0, 0.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(yaw));
    a->Visit(DRAKE_NVP(forward_speed));
    a->Visit(DRAKE_NVP(yaw_rate));
    a->Visit(DRAKE_NVP(wheel_angles));
  }
};

struct MovingTargetControllerParams {
  double throttle_gain{10.0};
  double turn_gain{6.0};
  double max_drive_torque{12.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(throttle_gain));
    a->Visit(DRAKE_NVP(turn_gain));
    a->Visit(DRAKE_NVP(max_drive_torque));
  }
};

struct MovingTargetTeleopParams {
  double publish_rate{20.0};
  double throttle_step{0.2};
  double turn_step{0.25};
  double decay_rate{1.5};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(publish_rate));
    a->Visit(DRAKE_NVP(throttle_step));
    a->Visit(DRAKE_NVP(turn_step));
    a->Visit(DRAKE_NVP(decay_rate));
  }
};

struct MovingTargetLcmChannels {
  std::string teleop_command{"MOVING_TARGET_TELEOP_CMD"};
  std::string actuation_command{"MOVING_TARGET_ACTUATION_CMD"};
  std::string state{"MOVING_TARGET_STATE"};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(teleop_command));
    a->Visit(DRAKE_NVP(actuation_command));
    a->Visit(DRAKE_NVP(state));
  }
};

struct ActorConfig {
  std::string actor_name{"moving_target"};
  std::string meshcat_root{"/actors/moving_target"};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(actor_name));
    a->Visit(DRAKE_NVP(meshcat_root));
  }
};

struct MovingTargetSimParams {
  std::string model{"UAV_models/moving_target/moving_target.urdf"};
  MovingTargetPlantParams plant;
  MovingTargetInitialState initial_state;
  MovingTargetControllerParams controller;
  MovingTargetTeleopParams teleop;
  MovingTargetLcmChannels lcm_channels;
  ActorConfig actor;
  double publish_rate{60.0};
  double realtime_rate{1.0};
  double sim_time{30.0};
  bool console_log{true};
  double console_period{0.25};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(model));
    a->Visit(DRAKE_NVP(plant));
    a->Visit(DRAKE_NVP(initial_state));
    a->Visit(DRAKE_NVP(controller));
    a->Visit(DRAKE_NVP(teleop));
    a->Visit(DRAKE_NVP(lcm_channels));
    a->Visit(DRAKE_NVP(actor));
    a->Visit(DRAKE_NVP(publish_rate));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(sim_time));
    a->Visit(DRAKE_NVP(console_log));
    a->Visit(DRAKE_NVP(console_period));
  }
};

Eigen::VectorXd MakeMovingTargetInitialStateVector(
    const MovingTargetInitialState& initial_state);

}  // namespace uav_delivery
