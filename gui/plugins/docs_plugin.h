#pragma once

#include "gui/plugins/gui_plugin.h"
#include <string>

namespace dairlib {
namespace simulation_gui {

class DocsPlugin : public GuiPlugin {
 public:
  DocsPlugin() = default;
  ~DocsPlugin() override = default;
  
  void Render() override;
  std::string GetName() const override { return "Documentation"; }
};

}  // namespace simulation_gui
}  // namespace dairlib
