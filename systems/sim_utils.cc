#include "systems/sim_utils.h"

#include <iostream>

namespace uav_delivery {
namespace systems {

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSigint(int) {
  g_stop_requested = 1;
}

ConsoleLogger::ConsoleLogger(int state_size, double period) {
  input_port_ = this->DeclareVectorInputPort(
      "state", drake::systems::BasicVector<double>(state_size))
                    .get_index();
  this->DeclarePeriodicPublishEvent(period, 0.0, &ConsoleLogger::Print);
}

drake::systems::EventStatus ConsoleLogger::Print(
    const drake::systems::Context<double>& context) const {
  const Eigen::VectorXd x = this->get_input_port(input_port_).Eval(context);
  std::cout << "t=" << context.get_time() << " p=[" << x(0) << ", " << x(1)
            << ", " << x(2) << "] v=[" << x(3) << ", " << x(4) << ", "
            << x(5) << "] omega=[" << x(15) << ", " << x(16) << ", "
            << x(17) << "]" << std::endl;
  return drake::systems::EventStatus::Succeeded();
}

SimTerminator::SimTerminator() {
  this->DeclarePerStepUnrestrictedUpdateEvent(&SimTerminator::CheckForStop);
}

drake::systems::EventStatus SimTerminator::CheckForStop(
    const drake::systems::Context<double>&,
    drake::systems::State<double>*) const {
  if (g_stop_requested) {
    return drake::systems::EventStatus::ReachedTermination(
        this, "SIGINT requested.");
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace uav_delivery
