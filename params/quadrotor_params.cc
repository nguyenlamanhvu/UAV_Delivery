#include "params/quadrotor_params.h"

#include <cmath>

namespace uav_delivery {
namespace {

Eigen::Matrix3d RotationMatrixFromRpy(const Eigen::Vector3d& rpy) {
  const double cr = std::cos(rpy.x());
  const double sr = std::sin(rpy.x());
  const double cp = std::cos(rpy.y());
  const double sp = std::sin(rpy.y());
  const double cy = std::cos(rpy.z());
  const double sy = std::sin(rpy.z());

  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
       -sp, cp * sr, cp * cr;
  return R;
}

}  // namespace

Eigen::VectorXd MakeInitialStateVector(const QuadrotorInitialState& initial_state) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(18);
  const Eigen::Matrix3d R = RotationMatrixFromRpy(initial_state.rpy);
  x.segment<3>(0) = initial_state.position;
  x.segment<3>(3) = initial_state.velocity;
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x.data() + 6) = R;
  x.segment<3>(15) = initial_state.body_angular_velocity;
  return x;
}

}  // namespace uav_delivery
