#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

namespace uav_delivery {

struct QuadrotorParams {
  double mass{0.775};
  double gravity{9.81};
  double arm_length{0.15};
  double thrust_coeff{1.0};
  double yaw_moment_coeff{1.0e-2};
  double max_rotor_input{5.0};
  Eigen::Vector3d inertia{0.0015, 0.0025, 0.0035};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mass));
    a->Visit(DRAKE_NVP(gravity));
    a->Visit(DRAKE_NVP(arm_length));
    a->Visit(DRAKE_NVP(thrust_coeff));
    a->Visit(DRAKE_NVP(yaw_moment_coeff));
    a->Visit(DRAKE_NVP(max_rotor_input));
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
  std::string reference{"UAV_QUADROTOR_REFERENCE"};
  std::string reference_trajectory{"UAV_QUADROTOR_REFERENCE_TRAJECTORY"};
  std::string sim_time{"UAV_SIM_TIME"};
  std::string setpoint{"UAV_QUADROTOR_SETPOINT"};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(command));
    a->Visit(DRAKE_NVP(state));
    a->Visit(DRAKE_NVP(reference));
    a->Visit(DRAKE_NVP(reference_trajectory));
    a->Visit(DRAKE_NVP(sim_time));
    a->Visit(DRAKE_NVP(setpoint));
  }
};

struct QuadrotorSe3ControllerParams {
  Eigen::Vector3d desired_position{0.0, 0.0, 1.0};
  Eigen::Vector3d desired_velocity{0.0, 0.0, 0.0};
  double desired_yaw{0.0};
  double kp_position{4.0};
  double kd_position{3.0};
  double kp_rotation{1.5};
  double kd_angular{0.3};

  double max_velocity{3.0};
  Eigen::Vector3d kv_velocity{3.0, 3.0, 4.0};
  Eigen::Vector3d ki_velocity{0.2, 0.2, 0.3};
  Eigen::Vector3d aerodynamic_drag{0.15, 0.15, 0.0};
  Eigen::Vector3d k_rp{6.0, 6.0, 0.0};
  double k_yaw{1.5};
  Eigen::Vector3d k_omega{0.35, 0.35, 0.3};
  double observer_gain{8.0};
  double observer_force_gain{4.0};
  double max_observer_force{8.0};
  double max_integral_speed{6.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(desired_position));
    a->Visit(DRAKE_NVP(desired_velocity));
    a->Visit(DRAKE_NVP(desired_yaw));
    a->Visit(DRAKE_NVP(kp_position));
    a->Visit(DRAKE_NVP(kd_position));
    a->Visit(DRAKE_NVP(kp_rotation));
    a->Visit(DRAKE_NVP(kd_angular));
    a->Visit(DRAKE_NVP(max_velocity));
    a->Visit(DRAKE_NVP(kv_velocity));
    a->Visit(DRAKE_NVP(ki_velocity));
    a->Visit(DRAKE_NVP(aerodynamic_drag));
    a->Visit(DRAKE_NVP(k_rp));
    a->Visit(DRAKE_NVP(k_yaw));
    a->Visit(DRAKE_NVP(k_omega));
    a->Visit(DRAKE_NVP(observer_gain));
    a->Visit(DRAKE_NVP(observer_force_gain));
    a->Visit(DRAKE_NVP(max_observer_force));
    a->Visit(DRAKE_NVP(max_integral_speed));
  }
};

struct QuadrotorWaypoint {
  double time{0.0};
  Eigen::Vector3d position{0.0, 0.0, 1.0};
  double yaw{0.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(time));
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(yaw));
  }
};

struct QuadrotorTrajectoryParams {
  double publish_rate{10.0};
  int preview_horizon{20};
  double preview_dt{0.001};
  std::vector<QuadrotorWaypoint> waypoints{
      {0.0, Eigen::Vector3d{0.0, 0.0, 1.0}, 0.0},
      {5.0, Eigen::Vector3d{1.0, 0.0, 1.0}, 0.0},
      {10.0, Eigen::Vector3d{1.0, 1.0, 1.0}, 0.0},
  };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(publish_rate));
    a->Visit(DRAKE_NVP(preview_horizon));
    a->Visit(DRAKE_NVP(preview_dt));
    a->Visit(DRAKE_NVP(waypoints));
  }
};

struct QuadrotorSimParams {
  std::string model{"UAV_models/skydio_2/quadrotor.urdf"};
  QuadrotorParams plant;
  QuadrotorInitialState initial_state;
  QuadrotorLcmChannels lcm_channels;
  QuadrotorSe3ControllerParams se3_controller;
  QuadrotorTrajectoryParams trajectory;
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
    a->Visit(DRAKE_NVP(se3_controller));
    a->Visit(DRAKE_NVP(trajectory));
    a->Visit(DRAKE_NVP(publish_rate));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(sim_time));
    a->Visit(DRAKE_NVP(console_log));
    a->Visit(DRAKE_NVP(console_period));
  }
};

Eigen::VectorXd MakeInitialStateVector(const QuadrotorInitialState& initial_state);

}  // namespace uav_delivery
