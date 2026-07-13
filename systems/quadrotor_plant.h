#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "params/quadrotor_params.h"

namespace uav_delivery {
namespace systems {

class QuadrotorPlant final : public drake::systems::LeafSystem<double> {
 public:
  explicit QuadrotorPlant(QuadrotorParams params);

  static constexpr int kStateSize = 12;

  const drake::systems::InputPort<double>& get_command_input_port() const;

 private:
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override;

  void CopyState(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* output) const;

  struct RotorWrench {
    double total_thrust{};
    Eigen::Vector3d moment_B{Eigen::Vector3d::Zero()};
  };

  RotorWrench CalcRotorWrench(const Eigen::Vector4d& rotor_speeds) const;
  static Eigen::Matrix3d RotationMatrixFromRpy(const Eigen::Vector3d& rpy);
  static Eigen::Vector3d EulerRatesFromBodyRates(const Eigen::Vector3d& rpy,
                                                 const Eigen::Vector3d& omega_B);

  const QuadrotorParams params_;
  drake::systems::InputPortIndex command_port_;
};

}  // namespace systems
}  // namespace uav_delivery
