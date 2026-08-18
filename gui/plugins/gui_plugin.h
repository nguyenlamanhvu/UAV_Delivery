#pragma once

#include <string>
#include "gui/robot_state_adapter.h"

namespace dairlib {
namespace simulation_gui {

// Base class for all modular GUI plugins that can be placed in a grid cell.
class GuiPlugin {
 public:
  virtual ~GuiPlugin() = default;

  // Called every frame to render the plugin's ImGui content.
  virtual void Render() = 0;

  // Called when new telemetry data arrives.
  virtual void Consume(const RobotStateSnapshot& snapshot) {}

  // Returns the name of the plugin instance.
  virtual std::string GetName() const = 0;
};

}  // namespace simulation_gui
}  // namespace dairlib
