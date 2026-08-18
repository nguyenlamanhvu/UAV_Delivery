#pragma once

#include "gui/plugins/gui_plugin.h"
#include <string>

namespace dairlib {
namespace simulation_gui {

class MeshcatPlugin : public GuiPlugin {
 public:
  MeshcatPlugin();
  void Render() override;
  std::string GetName() const override { return "MeshCat Control"; }

 private:
  std::string url_{"http://localhost:7080"};
  char url_buffer_[256];
};

}  // namespace simulation_gui
}  // namespace dairlib
