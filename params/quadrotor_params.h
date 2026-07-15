#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

namespace uav_delivery {

struct QuadrotorParams {
  double mass{1.2};
  double gravity{9.81};
  double arm_length{0.22};
  double thrust_coeff{1.0e-5};
  double yaw_moment_coeff{1.6e-7};
  double max_rotor_speed{1200.0};
  Eigen::Vector3d inertia{0.018, 0.018, 0.032};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mass));
    a->Visit(DRAKE_NVP(gravity));
    a->Visit(DRAKE_NVP(arm_length));
    a->Visit(DRAKE_NVP(thrust_coeff));
    a->Visit(DRAKE_NVP(yaw_moment_coeff));
    a->Visit(DRAKE_NVP(max_rotor_speed));
    a->Visit(DRAKE_NVP(inertia));
  }
};

struct QuadrotorInitialState {
  Eigen::Vector3d position{0.0, 0.0, 1.0};
  Eigen::Vector3d rpy{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity{0.0, 0.0, 0.0};
  Eigen::Vector3d body_angular_velocity{0.0, 0.0, 0.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(rpy));
    a->Visit(DRAKE_NVP(velocity));
    a->Visit(DRAKE_NVP(body_angular_velocity));
  }
};

struct QuadrotorLcmChannels {
  std::string command{"UAV_QUADROTOR_COMMAND"};
  std::string state{"UAV_QUADROTOR_STATE"};
  std::string sim_time{"UAV_SIM_TIME"};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(command));
    a->Visit(DRAKE_NVP(state));
    a->Visit(DRAKE_NVP(sim_time));
  }
};

struct QuadrotorHoverControllerParams {
  double desired_z{1.0};
  Eigen::Vector3d kp_rpy{3.0, 3.0, 1.2};
  Eigen::Vector3d kd_angular{0.35, 0.35, 0.18};
  double kp_z{2.0};
  double kd_z{0.8};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(desired_z));
    a->Visit(DRAKE_NVP(kp_rpy));
    a->Visit(DRAKE_NVP(kd_angular));
    a->Visit(DRAKE_NVP(kp_z));
    a->Visit(DRAKE_NVP(kd_z));
  }
};

struct QuadrotorSimParams {
  std::string model{"UAV_models/skydio_2/quadrotor.urdf"};
  QuadrotorParams plant;
  QuadrotorInitialState initial_state;
  QuadrotorLcmChannels lcm_channels;
  QuadrotorHoverControllerParams hover_controller;
  double publish_rate{100.0};
  double realtime_rate{1.0};
  double sim_time{10.0};
  bool console_log{true};
  double console_period{0.25};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(model));
    a->Visit(DRAKE_NVP(plant));
    a->Visit(DRAKE_NVP(initial_state));
    a->Visit(DRAKE_NVP(lcm_channels));
    a->Visit(DRAKE_NVP(hover_controller));
    a->Visit(DRAKE_NVP(publish_rate));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(sim_time));
    a->Visit(DRAKE_NVP(console_log));
    a->Visit(DRAKE_NVP(console_period));
  }
};

Eigen::VectorXd MakeInitialStateVector(const QuadrotorInitialState& initial_state);

double CalcHoverRotorSpeed(const QuadrotorParams& params);

}  // namespace uav_delivery
