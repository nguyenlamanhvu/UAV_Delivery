#include "gui/plugins/rtsp_plugin.h"
#include "imgui.h"
#include <GL/gl.h>
#include <cstring>
#include <iostream>
#include <dlfcn.h>

// Mock GStreamer types
typedef void GstSample;
typedef void GstCaps;
typedef void GstStructure;
typedef void GstBuffer;
typedef void GError;
typedef enum { GST_STATE_NULL = 1, GST_STATE_PLAYING = 4 } GstState;
typedef enum { GST_MAP_READ = 1 } GstMapFlags;
struct GstMapInfo {
  void *memory;
  int flags;
  uint8_t *data;
  size_t size;
  size_t maxsize;
  void *user_data[4];
  void *_gst_reserved[4];
};

static void* libgst = nullptr;
static void* libgstapp = nullptr;

static int (*p_gst_is_initialized)(void) = nullptr;
static void (*p_gst_init)(int*, char***) = nullptr;
static GstElement* (*p_gst_parse_launch)(const char*, GError**) = nullptr;
static GstElement* (*p_gst_bin_get_by_name)(GstElement*, const char*) = nullptr;
static int (*p_gst_element_set_state)(GstElement*, GstState) = nullptr;
static void (*p_gst_object_unref)(void*) = nullptr;
static GstSample* (*p_gst_app_sink_try_pull_sample)(GstElement*, uint64_t) = nullptr;
static GstCaps* (*p_gst_sample_get_caps)(GstSample*) = nullptr;
static GstStructure* (*p_gst_caps_get_structure)(GstCaps*, int) = nullptr;
static int (*p_gst_structure_get_int)(GstStructure*, const char*, int*) = nullptr;
static GstBuffer* (*p_gst_sample_get_buffer)(GstSample*) = nullptr;
static int (*p_gst_buffer_map)(GstBuffer*, GstMapInfo*, GstMapFlags) = nullptr;
static void (*p_gst_buffer_unmap)(GstBuffer*, GstMapInfo*) = nullptr;
static void (*p_gst_sample_unref)(GstSample*) = nullptr;
static void (*p_g_error_free)(GError*) = nullptr;

namespace dairlib {
namespace simulation_gui {

RtspPlugin::RtspPlugin() {
  std::strncpy(url_buffer_, url_.c_str(), sizeof(url_buffer_));
  
  if (!libgst) {
    libgst = dlopen("libgstreamer-1.0.so.0", RTLD_LAZY);
    libgstapp = dlopen("libgstapp-1.0.so.0", RTLD_LAZY);
    if (libgst && libgstapp) {
      p_gst_is_initialized = (int (*)(void))dlsym(libgst, "gst_is_initialized");
      p_gst_init = (void (*)(int*, char***))dlsym(libgst, "gst_init");
      p_gst_parse_launch = (GstElement* (*)(const char*, GError**))dlsym(libgst, "gst_parse_launch");
      p_gst_bin_get_by_name = (GstElement* (*)(GstElement*, const char*))dlsym(libgst, "gst_bin_get_by_name");
      p_gst_element_set_state = (int (*)(GstElement*, GstState))dlsym(libgst, "gst_element_set_state");
      p_gst_object_unref = (void (*)(void*))dlsym(libgst, "gst_object_unref");
      p_gst_app_sink_try_pull_sample = (GstSample* (*)(GstElement*, uint64_t))dlsym(libgstapp, "gst_app_sink_try_pull_sample");
      p_gst_sample_get_caps = (GstCaps* (*)(GstSample*))dlsym(libgst, "gst_sample_get_caps");
      p_gst_caps_get_structure = (GstStructure* (*)(GstCaps*, int))dlsym(libgst, "gst_caps_get_structure");
      p_gst_structure_get_int = (int (*)(GstStructure*, const char*, int*))dlsym(libgst, "gst_structure_get_int");
      p_gst_sample_get_buffer = (GstBuffer* (*)(GstSample*))dlsym(libgst, "gst_sample_get_buffer");
      p_gst_buffer_map = (int (*)(GstBuffer*, GstMapInfo*, GstMapFlags))dlsym(libgst, "gst_buffer_map");
      p_gst_buffer_unmap = (void (*)(GstBuffer*, GstMapInfo*))dlsym(libgst, "gst_buffer_unmap");
      p_gst_sample_unref = (void (*)(GstSample*))dlsym(libgst, "gst_sample_unref");
      
      void* libglib = dlopen("libglib-2.0.so.0", RTLD_LAZY);
      p_g_error_free = (void (*)(GError*))dlsym(libglib, "g_error_free");

      if (p_gst_is_initialized && !p_gst_is_initialized()) {
        p_gst_init(nullptr, nullptr);
      }
    } else {
      std::cerr << "Failed to load GStreamer dynamic libraries!" << std::endl;
    }
  }

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
  if (!libgst) return;

  std::string uri = std::string(url_buffer_);
  std::string launch_str = "uridecodebin uri=" + uri + " ! videoconvert ! video/x-raw,format=RGBA ! appsink name=mysink drop=true max-buffers=1";
  
  GError* error = nullptr;
  pipeline_ = p_gst_parse_launch(launch_str.c_str(), &error);
  
  if (error) {
    std::cerr << "Failed to parse pipeline." << std::endl;
    if (p_g_error_free) p_g_error_free(error);
    return;
  }

  appsink_ = p_gst_bin_get_by_name((GstElement*)pipeline_, "mysink");
  p_gst_element_set_state((GstElement*)pipeline_, GST_STATE_PLAYING);
  is_playing_ = true;
}

void RtspPlugin::StopStream() {
  if (pipeline_ && libgst) {
    p_gst_element_set_state((GstElement*)pipeline_, GST_STATE_NULL);
    p_gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  if (appsink_ && libgst) {
    p_gst_object_unref(appsink_);
    appsink_ = nullptr;
  }
  is_playing_ = false;
  frame_width_ = 0;
  frame_height_ = 0;
}

void RtspPlugin::UpdateTexture() {
  if (!appsink_ || !libgst) return;

  GstSample* sample = p_gst_app_sink_try_pull_sample((GstElement*)appsink_, 0);
  if (!sample) return;

  GstCaps* caps = p_gst_sample_get_caps(sample);
  if (caps && frame_width_ == 0) {
    GstStructure* s = p_gst_caps_get_structure(caps, 0);
    p_gst_structure_get_int(s, "width", &frame_width_);
    p_gst_structure_get_int(s, "height", &frame_height_);
  }

  GstBuffer* buffer = p_gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (p_gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, frame_width_, frame_height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, map.data);
    glBindTexture(GL_TEXTURE_2D, 0);
    p_gst_buffer_unmap(buffer, &map);
  }

  p_gst_sample_unref(sample);
}

void RtspPlugin::Render() {
  ImGui::Text("RTSP Camera Stream");
  ImGui::Separator();
  ImGui::Spacing();

  if (!libgst) {
    ImGui::TextColored(ImVec4(1,0,0,1), "Error: GStreamer libraries not found on system.");
    return;
  }

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
