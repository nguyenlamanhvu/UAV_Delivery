#include "systems/sysid_observer.h"
#include <iostream>

namespace uav_delivery {
namespace systems {

SysIdObserver::SysIdObserver(double dt) : dt_(dt) {
  this->DeclareVectorInputPort("command", drake::systems::BasicVector<double>(4));
  this->DeclareVectorInputPort("actual_throttle", drake::systems::BasicVector<double>(4));

  // State: previous actual_throttle (4)
  this->DeclareDiscreteState(4);

  this->DeclarePeriodicDiscreteUpdateEvent(dt_, 0., &SysIdObserver::EstimateParameters);

  // Initialize 4 RLS filters. 
  // Forgetting factor lam = 0.999, initial covariance delta = 1000.0
  for (int i = 0; i < 4; ++i) {
    rls_filters_.push_back(std::make_unique<rls_filter::RLSFilter<double, 1>>(0.999, 1000.0));
  }
}

void SysIdObserver::EstimateParameters(const drake::systems::Context<double>& context,
                                       drake::systems::DiscreteValues<double>* discrete_state) const {
  const auto& u = this->get_input_port(0).Eval(context);
  const auto& y = this->get_input_port(1).Eval(context);
  auto prev_y = context.get_discrete_state(0).value();

  double estimated_tau_sum = 0.0;
  int valid_motors = 0;

  for (int i = 0; i < 4; ++i) {
    // Model: y_{k+1} - y_k = (dt / tau) * (u_k - y_k)
    // Let b = dt / tau.
    // X = (u_k - y_k)
    // Y = y_{k+1} - y_k
    double X_val = u[i] - prev_y[i];
    double Y_val = y[i] - prev_y[i];

    // Only update if there is enough excitation to avoid numerical explosion
    if (std::abs(X_val) > 1e-4) {
      Eigen::Matrix<double, 1, 1> X;
      X << X_val;
      rls_filters_[i]->update(X, Y_val);
    }

    double b = rls_filters_[i]->estimatedCoefficients()(0);
    if (b > 1e-6) {
      double tau = dt_ / b;
      estimated_tau_sum += tau;
      valid_motors++;
    }
    
    // Update previous y
    discrete_state->get_mutable_value(0)[i] = y[i];
  }

  // Print the estimation every 0.5s roughly
  static int print_count = 0;
  if (++print_count % static_cast<int>(0.5 / dt_) == 0 && valid_motors > 0) {
    double avg_tau = estimated_tau_sum / valid_motors;
    std::cout << "[SysID Online] RLS Estimated Motor Time Constant (tau): " 
              << avg_tau << " seconds." << std::endl;
  }
}

}  // namespace systems
}  // namespace uav_delivery
