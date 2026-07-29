#include "systems/reference_trajectory_unpacker.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "drake/common/value.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace uav_delivery {
namespace systems {
namespace {

constexpr const char* kReferenceTrajectoryName = "quadrotor_se3_reference";

}  // namespace

ReferenceTrajectoryUnpacker::ReferenceTrajectoryUnpacker(
    QuadrotorSimParams params)
    : params_(std::move(params)) {
  state_port_ = this->DeclareAbstractInputPort(
      "quadrotor_state", drake::Value<lcmt_quadrotor_state>{})
                    .get_index();
  trajectory_port_ = this->DeclareAbstractInputPort(
      "quadrotor_reference_trajectory",
      drake::Value<lcmt_timestamped_saved_traj>{MakeDefaultTrajectory()})
                         .get_index();
  this->DeclareAbstractOutputPort(
      "lcmt_quadrotor_reference", &ReferenceTrajectoryUnpacker::CalcReference);
}

void ReferenceTrajectoryUnpacker::CalcReference(
    const drake::systems::Context<double>& context,
    lcmt_quadrotor_reference* output) const {
  const auto& state =
      this->get_input_port(state_port_).Eval<lcmt_quadrotor_state>(context);
  const auto& trajectory =
      this->get_input_port(trajectory_port_).Eval<lcmt_timestamped_saved_traj>(
          context);
  const double time = state.utime * 1e-6;

  const auto& saved = trajectory.saved_traj;
  for (int traj_index = 0; traj_index < saved.num_trajectories; ++traj_index) {
    if (saved.trajectory_names[traj_index] != kReferenceTrajectoryName) {
      continue;
    }
    const auto& block = saved.trajectories[traj_index];
    if (block.num_points <= 0 || block.num_datatypes < 10) {
      break;
    }

    int knot = 0;
    while (knot < block.num_points - 1 && block.time_vec[knot] < time) {
      ++knot;
    }

    output->utime = state.utime;
    for (int i = 0; i < 3; ++i) {
      output->position[i] = block.datapoints[i][knot];
      output->velocity[i] = block.datapoints[i + 3][knot];
      output->acceleration[i] = block.datapoints[i + 6][knot];
      output->body_angular_velocity[i] = 0.0;
      output->body_angular_acceleration[i] = 0.0;
    }
    const double yaw = block.datapoints[9][knot];
    const Eigen::Matrix3d R =
        drake::math::RollPitchYaw<double>(0.0, 0.0, yaw)
            .ToRotationMatrix()
            .matrix();
    for (int i = 0; i < 9; ++i) {
      output->rotation[i] = R(i / 3, i % 3);
    }
    return;
  }

  *output = MakeDefaultReference(time);
}

lcmt_quadrotor_reference ReferenceTrajectoryUnpacker::MakeDefaultReference(
    double time) const {
  lcmt_quadrotor_reference reference{};
  reference.utime = static_cast<int64_t>(std::llround(time * 1e6));
  for (int i = 0; i < 3; ++i) {
    reference.position[i] = params_.se3_controller.desired_position(i);
    reference.velocity[i] = params_.se3_controller.desired_velocity(i);
    reference.acceleration[i] = 0.0;
    reference.body_angular_velocity[i] = 0.0;
    reference.body_angular_acceleration[i] = 0.0;
  }
  const Eigen::Matrix3d R =
      drake::math::RollPitchYaw<double>(
          0.0, 0.0, params_.se3_controller.desired_yaw)
          .ToRotationMatrix()
          .matrix();
  for (int i = 0; i < 9; ++i) {
    reference.rotation[i] = R(i / 3, i % 3);
  }
  return reference;
}

lcmt_timestamped_saved_traj ReferenceTrajectoryUnpacker::MakeDefaultTrajectory()
    const {
  lcmt_timestamped_saved_traj trajectory{};
  trajectory.utime = 0;
  lcmt_trajectory_block block{};
  block.trajectory_name = kReferenceTrajectoryName;
  block.num_points = 1;
  block.num_datatypes = 10;
  block.time_vec = {0.0};
  block.datatypes = {"x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
                     "yaw"};
  block.datapoints.resize(block.num_datatypes, std::vector<double>(1, 0.0));
  for (int i = 0; i < 3; ++i) {
    block.datapoints[i][0] = params_.se3_controller.desired_position(i);
    block.datapoints[i + 3][0] = params_.se3_controller.desired_velocity(i);
  }
  block.datapoints[9][0] = params_.se3_controller.desired_yaw;
  trajectory.saved_traj.num_trajectories = 1;
  trajectory.saved_traj.trajectory_names = {kReferenceTrajectoryName};
  trajectory.saved_traj.trajectories = {block};
  return trajectory;
}

}  // namespace systems
}  // namespace uav_delivery
