#include "params/quadrotor_params.h"

#include <cmath>

namespace uav_delivery {

Eigen::VectorXd MakeInitialStateVector(const QuadrotorInitialState& initial_state) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
  x.segment<3>(0) = initial_state.position;
  x.segment<3>(3) = initial_state.rpy;
  x.segment<3>(6) = initial_state.velocity;
  x.segment<3>(9) = initial_state.body_angular_velocity;
  return x;
}

double CalcHoverRotorSpeed(const QuadrotorParams& params) {
  return std::sqrt(params.mass * params.gravity / (4.0 * params.thrust_coeff));
}

}  // namespace uav_delivery
