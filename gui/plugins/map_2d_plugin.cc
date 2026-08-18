#include "gui/plugins/map_2d_plugin.h"
#include "imgui.h"
#include "implot.h"
#define STB_IMAGE_IMPLEMENTATION
#include "gui/plugins/stb_image.h"

// Note: Requires OpenGL headers for texture creation. Assuming gl.h is available via ImGui backend.
#include <GL/gl.h>

namespace dairlib {
namespace simulation_gui {

Map2DPlugin::Map2DPlugin() {
  LoadTexture();
}

Map2DPlugin::~Map2DPlugin() {
  if (texture_id_ != 0) {
    glDeleteTextures(1, &texture_id_);
  }
}

void Map2DPlugin::LoadTexture() {
  int channels;
  unsigned char* data = stbi_load("maps/map_2d_clean_no_trees_no_shadows.png", 
                                  &image_width_, &image_height_, &channels, 4);
  if (data == nullptr) {
    return;
  }

  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width_, image_height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
  stbi_image_free(data);
}

const double* Map2DPlugin::FindValue(const RobotStateSnapshot& snapshot, const std::string& field_path, const std::string& name) const {
  for (const auto& group : snapshot.signal_groups) {
    if (group.field_path != field_path) continue;
    for (size_t i = 0; i < group.names.size(); ++i) {
      if (group.names[i] == name && i < group.values.size()) {
        return &group.values[i];
      }
    }
  }
  return nullptr;
}

void Map2DPlugin::Consume(const RobotStateSnapshot& snapshot) {
  // Try to find quadrotor position
  if (auto* x = FindValue(snapshot, "position", "x")) drone_x_ = *x;
  if (auto* y = FindValue(snapshot, "position", "y")) drone_y_ = *y;
  
  // Try to find moving target position (assuming different source or unique group logic might apply, 
  // but if both emit "position", this gets the last one. Ideally we check source).
  // For simplicity, we just scan for a known target source if available, but assuming it emits "target_position" or similar.
  // We'll just read "position" from the target state:
  for (const auto& group : snapshot.signal_groups) {
    if (group.source.find("TARGET") != std::string::npos && group.field_path == "position") {
      for (size_t i = 0; i < group.names.size(); ++i) {
        if (group.names[i] == "x") target_x_ = group.values[i];
        if (group.names[i] == "y") target_y_ = group.values[i];
      }
    } else if (group.source.find("QUADROTOR") != std::string::npos && group.field_path == "position") {
      for (size_t i = 0; i < group.names.size(); ++i) {
        if (group.names[i] == "x") drone_x_ = group.values[i];
        if (group.names[i] == "y") drone_y_ = group.values[i];
      }
    }
  }
}

void Map2DPlugin::Render() {
  if (texture_id_ == 0) {
    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Failed to load map image.");
    return;
  }

  // Settings panel
  ImGui::InputDouble("Min X", &map_min_x_); ImGui::SameLine(); ImGui::InputDouble("Max X", &map_max_x_);
  ImGui::InputDouble("Min Y", &map_min_y_); ImGui::SameLine(); ImGui::InputDouble("Max Y", &map_max_y_);
  ImGui::Text("Drone: (%.2f, %.2f) | Target: (%.2f, %.2f)", drone_x_, drone_y_, target_x_, target_y_);

  if (ImPlot::BeginPlot("##map_plot", ImVec2(-1, -1), ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("X (m)", "Y (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, map_min_x_, map_max_x_);
    ImPlot::SetupAxisLimits(ImAxis_Y1, map_min_y_, map_max_y_);
    
    // Plot the background image
    ImPlot::PlotImage("Map", (void*)(intptr_t)texture_id_, 
                      ImPlotPoint(map_min_x_, map_min_y_), 
                      ImPlotPoint(map_max_x_, map_max_y_));

    // Plot Drone (Triangle)
    double dx[1] = { drone_x_ };
    double dy[1] = { drone_y_ };
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Up, 12, ImVec4(0, 0.8f, 1, 1), 2.0f, ImVec4(0, 0.5f, 0.8f, 1));
    ImPlot::PlotScatter("Drone", dx, dy, 1);

    // Plot Target (Triangle)
    double tx[1] = { target_x_ };
    double ty[1] = { target_y_ };
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Down, 12, ImVec4(1, 0.2f, 0, 1), 2.0f, ImVec4(0.8f, 0.1f, 0, 1));
    ImPlot::PlotScatter("Target", tx, ty, 1);

    if (ImPlot::IsPlotHovered()) {
        ImPlotPoint mouse = ImPlot::GetPlotMousePos();
        ImGui::SetTooltip("Map Coordinate: X: %.2f, Y: %.2f", mouse.x, mouse.y);
    }

    ImPlot::EndPlot();
  }
}

}  // namespace simulation_gui
}  // namespace dairlib
