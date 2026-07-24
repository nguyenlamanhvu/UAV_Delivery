#include "systems/waypoint_trajectory_source.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "drake/common/value.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace uav_delivery {
namespace systems {
namespace {

constexpr const char* kReferenceTrajectoryName = "quadrotor_se3_reference";

double Clamp01(double x) {
  return std::clamp(x, 0.0, 1.0);
}

double WrapToPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

WaypointTrajectorySource::WaypointTrajectorySource(
    QuadrotorTrajectoryParams params)
    : params_(std::move(params)), waypoints_(params_.waypoints) {
  if (waypoints_.size() < 2) {
    throw std::runtime_error("trajectory.waypoints must contain at least 2 points");
  }
  std::sort(waypoints_.begin(), waypoints_.end(),
            [](const QuadrotorWaypoint& a, const QuadrotorWaypoint& b) {
              return a.time < b.time;
            });
  for (int i = 1; i < static_cast<int>(waypoints_.size()); ++i) {
    if (waypoints_[i].time <= waypoints_[i - 1].time) {
      throw std::runtime_error("trajectory.waypoints times must be increasing");
    }
  }

  state_port_ = this->DeclareAbstractInputPort(
      "quadrotor_state", drake::Value<lcmt_quadrotor_state>{})
                    .get_index();
  reference_state_index_ = this->DeclareAbstractState(
      drake::Value<lcmt_timestamped_saved_traj>{
          MakeReferenceTrajectory(waypoints_.front().time)});
  this->DeclarePeriodicUnrestrictedUpdateEvent(
      1.0 / params_.publish_rate, 0.0,
      &WaypointTrajectorySource::UpdateReference);
  this->DeclareAbstractOutputPort("lcmt_quadrotor_reference",
                                  &WaypointTrajectorySource::CalcReference);
}

drake::systems::EventStatus WaypointTrajectorySource::UpdateReference(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {
  auto& reference =
      state->get_mutable_abstract_state<lcmt_timestamped_saved_traj>(
          reference_state_index_);
  reference = MakeReferenceTrajectory(context.get_time());
  return drake::systems::EventStatus::Succeeded();
}

void WaypointTrajectorySource::CalcReference(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* output) const {
  *output = context.get_abstract_state<lcmt_timestamped_saved_traj>(
      reference_state_index_);
}

lcmt_timestamped_saved_traj WaypointTrajectorySource::MakeReferenceTrajectory(
    double time) const {
  lcmt_timestamped_saved_traj output{};
  output.utime = static_cast<int64_t>(std::llround(time * 1e6));

  lcmt_trajectory_block block{};
  block.trajectory_name = kReferenceTrajectoryName;
  block.num_points = params_.preview_horizon;
  block.num_datatypes = 10;
  block.datatypes = {"x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
                     "yaw"};
  block.time_vec.resize(block.num_points);
  block.datapoints.resize(
      block.num_datatypes, std::vector<double>(block.num_points, 0.0));

  for (int knot = 0; knot < block.num_points; ++knot) {
    const double knot_time = time + knot * params_.preview_dt;
    const SegmentReference ref = Evaluate(knot_time);
    block.time_vec[knot] = knot_time;
    for (int i = 0; i < 3; ++i) {
      block.datapoints[i][knot] = ref.position(i);
      block.datapoints[i + 3][knot] = ref.velocity(i);
      block.datapoints[i + 6][knot] = ref.acceleration(i);
    }
    block.datapoints[9][knot] = ref.yaw;
  }

  output.saved_traj.num_trajectories = 1;
  output.saved_traj.trajectory_names = {kReferenceTrajectoryName};
  output.saved_traj.trajectories = {block};
  return output;
}

WaypointTrajectorySource::SegmentReference WaypointTrajectorySource::Evaluate(
    double time) const {
  if (time <= waypoints_.front().time) {
    return {.position = waypoints_.front().position,
            .yaw = waypoints_.front().yaw};
  }
  if (time >= waypoints_.back().time) {
    return {.position = waypoints_.back().position,
            .yaw = waypoints_.back().yaw};
  }

  auto upper = std::upper_bound(
      waypoints_.begin(), waypoints_.end(), time,
      [](double t, const QuadrotorWaypoint& waypoint) {
        return t < waypoint.time;
      });
  const QuadrotorWaypoint& b = *upper;
  const QuadrotorWaypoint& a = *(upper - 1);
  const double duration = b.time - a.time;
  const double tau = Clamp01((time - a.time) / duration);
  const double s = 3.0 * tau * tau - 2.0 * tau * tau * tau;
  const double s_dot = (6.0 * tau - 6.0 * tau * tau) / duration;
  const double s_ddot = (6.0 - 12.0 * tau) / (duration * duration);

  const Eigen::Vector3d dp = b.position - a.position;
  const double dyaw = WrapToPi(b.yaw - a.yaw);
  return {.position = a.position + s * dp,
          .velocity = s_dot * dp,
          .acceleration = s_ddot * dp,
          .yaw = a.yaw + s * dyaw,
          .yaw_rate = s_dot * dyaw,
          .yaw_acceleration = s_ddot * dyaw};
}

}  // namespace systems
}  // namespace uav_delivery
