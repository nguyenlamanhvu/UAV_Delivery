#include "params/moving_target_params.h"

namespace uav_delivery {

Eigen::VectorXd MakeMovingTargetInitialStateVector(
    const MovingTargetInitialState& initial_state) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
  x(0) = initial_state.position.x();
  x(1) = initial_state.position.y();
  x(2) = initial_state.yaw;
  x(3) = initial_state.forward_speed;
  x(4) = initial_state.yaw_rate;
  for (int i = 0; i < 4; ++i) {
    x(5 + i) = initial_state.wheel_angles[static_cast<std::size_t>(i)];
  }
  return x;
}

}  // namespace uav_delivery
