#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <Eigen/Dense>
#include <vector>

#include "lcmtypes/uav_delivery/lcmt_quadrotor_state.hpp"
#include "lcmtypes/uav_delivery/lcmt_quadrotor_setpoint.hpp"
#include "params/quadrotor_params.h"

namespace uav_delivery {
namespace systems {

class NmpcController : public drake::systems::LeafSystem<double> {
 public:
  explicit NmpcController(QuadrotorSimParams params);
  const QuadrotorSimParams& get_params() const { return params_; }

 private:
  void CalcCommand(const drake::systems::Context<double>& context,
                   lcmt_quadrotor_setpoint* output) const;
                   
  Eigen::VectorXd Dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
  
  void CalcDerivatives(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::VectorXd& x_ref,
                       Eigen::MatrixXd& A, Eigen::MatrixXd& B, 
                       Eigen::VectorXd& l_x, Eigen::VectorXd& l_u,
                       Eigen::MatrixXd& l_xx, Eigen::MatrixXd& l_uu) const;
                       
  Eigen::Vector3d SolveMPC(const Eigen::VectorXd& x0, const Eigen::VectorXd& x_ref) const;

  int state_port_{-1};
  int setpoint_port_{-1};
  QuadrotorSimParams params_;
  
  static constexpr int kNumStates = 6; // [p(3), v(3)]
  static constexpr int kNumInputs = 3; // [a(3)]
  static constexpr int kHorizon = 40; 
  static constexpr double kDt = 0.02; // 50 Hz internal planner
  
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  
  mutable std::vector<Eigen::VectorXd> x_traj_;
  mutable std::vector<Eigen::VectorXd> u_traj_;
};

}  // namespace systems
}  // namespace uav_delivery
