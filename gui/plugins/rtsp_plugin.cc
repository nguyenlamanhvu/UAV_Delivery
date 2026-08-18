#include "gui/plugins/rtsp_plugin.h"
#include "imgui.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <GL/gl.h>
#include <cstring>
#include <iostream>

namespace dairlib {
namespace simulation_gui {

RtspPlugin::RtspPlugin() {
  std::strncpy(url_buffer_, url_.c_str(), sizeof(url_buffer_));
  
  // Initialization is now done in main() for safety
  // to avoid GLib assertion crashes.

  // Create an empty texture
  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);
}

RtspPlugin::~RtspPlugin() {
  StopStream();
  if (texture_id_ != 0) {
    glDeleteTextures(1, &texture_id_);
  }
}

void RtspPlugin::StartStream() {
  if (is_playing_) StopStream();

  std::string uri = std::string(url_buffer_);
  std::string launch_str = "uridecodebin uri=" + uri + " ! videoconvert ! video/x-raw,format=RGBA ! appsink name=mysink drop=true max-buffers=1";
  
  GError* error = nullptr;
  pipeline_ = gst_parse_launch(launch_str.c_str(), &error);
  
  if (error) {
    std::cerr << "Failed to parse pipeline: " << error->message << std::endl;
    g_error_free(error);
    return;
  }

  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
  
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  is_playing_ = true;
}

void RtspPlugin::StopStream() {
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  if (appsink_) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }
  is_playing_ = false;
  frame_width_ = 0;
  frame_height_ = 0;
}

void RtspPlugin::UpdateTexture() {
  if (!appsink_) return;

  // Try to pull a sample without blocking
  GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
  if (!sample) return;

  GstCaps* caps = gst_sample_get_caps(sample);
  if (caps && frame_width_ == 0) {
    GstStructure* s = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(s, "width", &frame_width_);
    gst_structure_get_int(s, "height", &frame_height_);
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    // We expect RGBA 8-bit per channel
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, frame_width_, frame_height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, map.data);
    glBindTexture(GL_TEXTURE_2D, 0);
    gst_buffer_unmap(buffer, &map);
  }

  gst_sample_unref(sample);
}

void RtspPlugin::Render() {
  ImGui::Text("RTSP Camera Stream");
  ImGui::Separator();
  ImGui::Spacing();

  ImGui::InputText("RTSP URL", url_buffer_, sizeof(url_buffer_));
  
  if (is_playing_) {
    if (ImGui::Button("Stop Stream", ImVec2(120, 30))) {
      StopStream();
    }
  } else {
    if (ImGui::Button("Start Stream", ImVec2(120, 30))) {
      StartStream();
    }
  }

  ImGui::Spacing();
  
  if (is_playing_) {
    UpdateTexture();
    if (frame_width_ > 0 && frame_height_ > 0) {
      // Calculate aspect ratio
      float aspect = (float)frame_width_ / (float)frame_height_;
      float available_width = ImGui::GetContentRegionAvail().x;
      float render_height = available_width / aspect;
      
      ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(available_width, render_height));
    } else {
      ImGui::TextDisabled("Connecting to stream / waiting for frames...");
    }
  } else {
    ImGui::TextDisabled("Stream stopped.");
  }
}

}  // namespace simulation_gui
}  // namespace dairlib
