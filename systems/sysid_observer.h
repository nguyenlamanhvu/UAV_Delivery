#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "RLSFilter.h"
#include <memory>
#include <vector>

namespace uav_delivery {
namespace systems {

class SysIdObserver : public drake::systems::LeafSystem<double> {
 public:
  SysIdObserver(double dt);

 private:
  void EstimateParameters(const drake::systems::Context<double>& context,
                          drake::systems::DiscreteValues<double>* discrete_state) const;

  double dt_;
  
  // We use 4 RLS filters, one for each motor.
  // We dynamically allocate them because RLSFilter does not have a default constructor.
  // The state will hold the previous 'actual_throttle' to compute the derivative.
  mutable std::vector<std::unique_ptr<rls_filter::RLSFilter<double, 1>>> rls_filters_;
};

}  // namespace systems
}  // namespace uav_delivery
