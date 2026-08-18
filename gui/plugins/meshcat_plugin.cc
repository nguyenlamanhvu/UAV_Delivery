#include "gui/plugins/meshcat_plugin.h"
#include "imgui.h"
#include <cstdlib>
#include <cstring>

namespace dairlib {
namespace simulation_gui {

MeshcatPlugin::MeshcatPlugin() {
  std::strncpy(url_buffer_, url_.c_str(), sizeof(url_buffer_));
}

void MeshcatPlugin::Render() {
  ImGui::Text("MeshCat 3D Visualizer");
  ImGui::Separator();
  ImGui::Spacing();
  
  ImGui::Text("MeshCat is optimized and running in the background.");
  ImGui::Text("Use the button below to open the full 3D interactive viewer.");
  ImGui::Spacing();

  ImGui::InputText("URL", url_buffer_, sizeof(url_buffer_));
  
  if (ImGui::Button("Open MeshCat in Browser", ImVec2(-1, 40))) {
    std::string url(url_buffer_);
    std::string command = "xdg-open " + url + " &";
    int ret = std::system(command.c_str()); (void)ret;
  }
}

}  // namespace simulation_gui
}  // namespace dairlib
