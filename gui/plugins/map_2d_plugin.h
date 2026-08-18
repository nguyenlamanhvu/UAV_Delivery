#pragma once

#include "gui/plugins/gui_plugin.h"
#include <string>

namespace dairlib {
namespace simulation_gui {

class Map2DPlugin : public GuiPlugin {
 public:
  Map2DPlugin();
  ~Map2DPlugin() override;
  
  void Render() override;
  void Consume(const RobotStateSnapshot& snapshot) override;
  std::string GetName() const override { return "2D Map Navigation"; }

 private:
  void LoadTexture();
  const double* FindValue(const RobotStateSnapshot& snapshot, const std::string& field_path, const std::string& name) const;

  unsigned int texture_id_{0};
  int image_width_{0};
  int image_height_{0};
  
  double drone_x_{0.0}, drone_y_{0.0};
  double target_x_{0.0}, target_y_{0.0};
  
  // Real-world map bounds (adjust these to match your actual map's scale in meters)
  double map_min_x_{-50.0}, map_max_x_{50.0};
  double map_min_y_{-50.0}, map_max_y_{50.0};
};

}  // namespace simulation_gui
}  // namespace dairlib
