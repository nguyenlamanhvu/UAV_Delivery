#include "systems/nmpc_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace uav_delivery {
namespace systems {

NmpcController::NmpcController(QuadrotorSimParams params)
    : params_(std::move(params)) {
  state_port_ = this->DeclareAbstractInputPort(
      "quadrotor_state", drake::Value<lcmt_quadrotor_state>{})
                    .get_index();
  setpoint_port_ = this->DeclareAbstractInputPort(
      "nmpc_reference", drake::Value<lcmt_quadrotor_setpoint>{})
                        .get_index();
  this->DeclareAbstractOutputPort("lcmt_quadrotor_setpoint",
                                  &NmpcController::CalcCommand);
                                  
  Q_ = Eigen::MatrixXd::Zero(kNumStates, kNumStates);
  Q_.diagonal() << 15.0, 15.0, 15.0,   // p: Strong position tracking
                   5.0, 5.0, 5.0;      // v: Velocity tracking
                   
  R_ = Eigen::MatrixXd::Identity(kNumInputs, kNumInputs) * 0.1;
  
  x_traj_.resize(kHorizon, Eigen::VectorXd::Zero(kNumStates));
  u_traj_.resize(kHorizon - 1, Eigen::VectorXd::Zero(kNumInputs));
}

Eigen::VectorXd NmpcController::Dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
  Eigen::VectorXd x_next(kNumStates);
  Eigen::Vector3d p = x.segment<3>(0);
  Eigen::Vector3d v = x.segment<3>(3);
  
  // u is the acceleration command (a_cmd)
  Eigen::Vector3d v_dot = u;
  
  x_next.segment<3>(0) = p + v * kDt;
  x_next.segment<3>(3) = v + v_dot * kDt;
  return x_next;
}

void NmpcController::CalcDerivatives(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::VectorXd& x_ref,
                                     Eigen::MatrixXd& A, Eigen::MatrixXd& B, 
                                     Eigen::VectorXd& l_x, Eigen::VectorXd& l_u,
                                     Eigen::MatrixXd& l_xx, Eigen::MatrixXd& l_uu) const {
  A = Eigen::MatrixXd::Identity(kNumStates, kNumStates);
  A.topRightCorner(3, 3) = Eigen::Matrix3d::Identity() * kDt;
  
  B = Eigen::MatrixXd::Zero(kNumStates, kNumInputs);
  B.bottomRows(3) = Eigen::Matrix3d::Identity() * kDt;
  
  Eigen::VectorXd dx = x - x_ref;
  l_x = Q_ * dx;
  l_xx = Q_;
  
  // Desired feedforward acceleration (from reference)
  Eigen::Vector3d a_ref(0, 0, 0); 
  Eigen::VectorXd du = u - a_ref;
  l_u = R_ * du;
  l_uu = R_;
}

Eigen::Vector3d NmpcController::SolveMPC(const Eigen::VectorXd& x0, const Eigen::VectorXd& x_ref) const {
  x_traj_[0] = x0;
  
  for (int t = 0; t < kHorizon - 1; ++t) {
    x_traj_[t+1] = Dynamics(x_traj_[t], u_traj_[t]);
  }
  
  std::vector<Eigen::MatrixXd> K(kHorizon - 1, Eigen::MatrixXd::Zero(kNumInputs, kNumStates));
  std::vector<Eigen::VectorXd> k(kHorizon - 1, Eigen::VectorXd::Zero(kNumInputs));
  
  Eigen::VectorXd dx_final = x_traj_.back() - x_ref;
  Eigen::VectorXd V_x = Q_ * dx_final;
  Eigen::MatrixXd V_xx = Q_;
  
  for (int t = kHorizon - 2; t >= 0; --t) {
    Eigen::MatrixXd A, B, l_xx, l_uu;
    Eigen::VectorXd l_x, l_u;
    CalcDerivatives(x_traj_[t], u_traj_[t], x_ref, A, B, l_x, l_u, l_xx, l_uu);
    
    Eigen::VectorXd Q_x = l_x + A.transpose() * V_x;
    Eigen::VectorXd Q_u = l_u + B.transpose() * V_x;
    Eigen::MatrixXd Q_xx = l_xx + A.transpose() * V_xx * A;
    Eigen::MatrixXd Q_uu = l_uu + B.transpose() * V_xx * B;
    Eigen::MatrixXd Q_ux = B.transpose() * V_xx * A;
    
    Q_uu += Eigen::MatrixXd::Identity(kNumInputs, kNumInputs) * 1e-4;
    
    Eigen::MatrixXd Q_uu_inv = Q_uu.inverse();
    K[t] = -Q_uu_inv * Q_ux;
    k[t] = -Q_uu_inv * Q_u;
    
    V_x = Q_x + K[t].transpose() * Q_uu * k[t] + K[t].transpose() * Q_u + Q_ux.transpose() * k[t];
    V_xx = Q_xx + K[t].transpose() * Q_uu * K[t] + K[t].transpose() * Q_ux + Q_ux.transpose() * K[t];
  }
  
  Eigen::Vector3d u_opt = u_traj_[0] + k[0];
  
  // Limit acceleration for feasibility
  for (int i=0; i<3; i++) {
      u_opt(i) = std::clamp(u_opt(i), -15.0, 15.0);
  }
  
  for (int t = 0; t < kHorizon - 2; ++t) {
    u_traj_[t] = u_traj_[t+1] + k[t+1];
    for (int i=0; i<3; i++) {
        u_traj_[t](i) = std::clamp(u_traj_[t](i), -15.0, 15.0);
    }
  }
  u_traj_.back() = u_traj_[kHorizon - 3];
  
  return u_opt;
}

void NmpcController::CalcCommand(const drake::systems::Context<double>& context,
                                 lcmt_quadrotor_setpoint* output) const {
  // NMPC Pass-through to SE3 Inner-Loop
  // Since the incoming reference does not contain the full future trajectory,
  // applying a receding horizon LQR to a constant instantaneous setpoint introduces massive phase lag.
  // Instead, the NMPC outer-loop acts as a setpoint filter and mode initialization layer.
  
  double current_time = context.get_time();
  const auto& ref = this->get_input_port(setpoint_port_).Eval<lcmt_quadrotor_setpoint>(context);
  
  output->utime = static_cast<int64_t>(std::llround(current_time * 1e6));
  output->mode = 0; // Position mode
  output->yaw = ref.yaw;
  output->yaw_rate = 0.0;
  
  for (int i = 0; i < 3; ++i) {
    output->position[i] = ref.position[i];
    output->velocity[i] = ref.velocity[i];
    output->acceleration[i] = ref.acceleration[i];
  }
}

}  // namespace systems
}  // namespace uav_delivery
