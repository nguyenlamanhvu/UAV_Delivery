#include "gui/plugins/docs_plugin.h"
#include "imgui.h"
#include <cstdlib>

namespace dairlib {
namespace simulation_gui {

void DocsPlugin::Render() {
  ImGui::Text("Project Documentation");
  ImGui::Separator();
  ImGui::Spacing();
  
  ImGui::TextWrapped("Click the button below to open the project README.md in your default text editor or viewer.");
  ImGui::Spacing();

  if (ImGui::Button("Open README.md", ImVec2(-1, 40))) {
    // Attempt to open the README in the project root
    // xdg-open will use the system default application for markdown files
    std::system("xdg-open README.md &");
  }
}

}  // namespace simulation_gui
}  // namespace dairlib
