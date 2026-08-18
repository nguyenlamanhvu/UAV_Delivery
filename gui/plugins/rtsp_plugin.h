#pragma once

#include "gui/plugins/gui_plugin.h"
#include <string>

// Forward declarations to avoid exposing GStreamer headers here
typedef struct _GstElement GstElement;

namespace dairlib {
namespace simulation_gui {

class RtspPlugin : public GuiPlugin {
 public:
  RtspPlugin();
  ~RtspPlugin() override;
  
  void Render() override;
  std::string GetName() const override { return "RTSP Camera Stream"; }

 private:
  void StartStream();
  void StopStream();
  void UpdateTexture();

  std::string url_{"rtsp://127.0.0.1:8554/test"};
  char url_buffer_[256];
  
  bool is_playing_{false};
  GstElement* pipeline_{nullptr};
  GstElement* appsink_{nullptr};

  unsigned int texture_id_{0};
  int frame_width_{0};
  int frame_height_{0};
};

}  // namespace simulation_gui
}  // namespace dairlib
