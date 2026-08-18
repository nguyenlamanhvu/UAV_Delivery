#pragma once

#include <memory>
#include <string>

#include "gui/robot_state_adapter.h"

namespace dairlib {
namespace simulation_gui {

// Hierarchical signal browser and independent multi-curve plot tabs.
class SignalWorkspace {
 public:
  SignalWorkspace(std::string transport_label, std::string source_description,
                  std::string renderer, int max_points, bool light_theme,
                  double history_seconds, double gap_threshold_seconds);
  ~SignalWorkspace();

  SignalWorkspace(const SignalWorkspace&) = delete;
  SignalWorkspace& operator=(const SignalWorkspace&) = delete;

  void Consume(RobotStateSnapshot snapshot);
  void Render(RobotStateAdapter* adapter);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace simulation_gui
}  // namespace dairlib
