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
      drake::Value<lcmt_quadrotor_reference>{MakeReference(waypoints_.front().time)});
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
      state->get_mutable_abstract_state<lcmt_quadrotor_reference>(
          reference_state_index_);
  reference = MakeReference(context.get_time());
  return drake::systems::EventStatus::Succeeded();
}

void WaypointTrajectorySource::CalcReference(
    const drake::systems::Context<double>& context,
    lcmt_quadrotor_reference* output) const {
  *output = context.get_abstract_state<lcmt_quadrotor_reference>(
      reference_state_index_);
}

lcmt_quadrotor_reference WaypointTrajectorySource::MakeReference(
    double time) const {
  lcmt_quadrotor_reference output{};
  const SegmentReference ref = Evaluate(time);
  output.utime = static_cast<int64_t>(std::llround(time * 1e6));

  const Eigen::Matrix3d R =
      drake::math::RollPitchYaw<double>(0.0, 0.0, ref.yaw)
          .ToRotationMatrix()
          .matrix();
  for (int i = 0; i < 3; ++i) {
    output.position[i] = ref.position(i);
    output.velocity[i] = ref.velocity(i);
    output.acceleration[i] = ref.acceleration(i);
    output.body_angular_velocity[i] = 0.0;
    output.body_angular_acceleration[i] = 0.0;
  }
  for (int i = 0; i < 9; ++i) {
    output.rotation[i] = R(i / 3, i % 3);
  }
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
