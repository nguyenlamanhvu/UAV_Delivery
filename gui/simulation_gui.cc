// Live robot-state plots for simulation and AI debugging.

#include <cstdio>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "gui/robot_state_adapter.h"
#include "gui/signal_workspace.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=2",
              "LCM URL used by franka_sim.");
DEFINE_string(lcm_channels, "",
              "Optional comma-separated lcmt_robot_output channel override; "
              "empty discovers compatible channels.");
DEFINE_string(channel, "",
              "Optional single LCM channel; overrides --lcm_channels.");
DEFINE_string(transport, "lcm",
              "Robot-state transport: ros2, fastdds, or lcm.");
DEFINE_string(ros2_topic, "",
              "Optional ROS 2 control_ai_msgs/msg/RobotStates topic override; "
              "empty discovers compatible topics.");
DEFINE_string(
    fastdds_topic, "",
    "Optional raw DDS topic override, such as rt/robot_states/robot_states.");
DEFINE_string(theme, "light", "GUI color theme: light or dark.");
DEFINE_double(sample_rate_hz, 120.0,
              "Maximum robot-state rate retained by the adapter.");
DEFINE_int32(max_pending_samples, 2400,
             "Maximum decimated samples buffered across render stalls.");
DEFINE_double(gap_threshold_ms, 100.0,
              "Timestamp gap that breaks the plotted line.");
DEFINE_int32(max_points, 6000,
             "Maximum number of samples retained in each plotted curve.");
DEFINE_double(history_seconds, 10.0,
              "Initial visible history for newly created plot tabs.");

namespace dairlib {
namespace {

using simulation_gui::MakeRobotStateAdapter;
using simulation_gui::RobotStateAdapterConfig;
using simulation_gui::SignalWorkspace;

std::vector<std::string> ParseChannelList(const std::string& value) {
  std::vector<std::string> channels;
  std::istringstream stream(value);
  for (std::string item; std::getline(stream, item, ',');) {
    const size_t first = item.find_first_not_of(" \t");
    if (first == std::string::npos) continue;
    const size_t last = item.find_last_not_of(" \t");
    channels.push_back(item.substr(first, last - first + 1));
  }
  return channels;
}

int DoMain() {
  bool light_theme = false;
  if (FLAGS_theme == "light") {
    light_theme = true;
  } else if (FLAGS_theme != "dark") {
    std::fprintf(stderr, "Unknown --theme='%s' (expected light or dark)\n",
                 FLAGS_theme.c_str());
    return 1;
  }

  if (FLAGS_sample_rate_hz <= 0.0 || FLAGS_max_pending_samples <= 0 ||
      FLAGS_gap_threshold_ms <= 0.0 || FLAGS_max_points <= 1 ||
      FLAGS_history_seconds <= 0.0) {
    std::fprintf(
        stderr,
        "Sampling, buffering, gap, history, and plot limits must be positive\n");
    return 1;
  }

  std::string transport_error_message;
  RobotStateAdapterConfig adapter_config;
  adapter_config.transport = FLAGS_transport;
  adapter_config.lcm_url = FLAGS_lcm_url;
  adapter_config.lcm_channels = FLAGS_channel.empty()
                                    ? ParseChannelList(FLAGS_lcm_channels)
                                    : std::vector<std::string>{FLAGS_channel};
  adapter_config.ros2_topic = FLAGS_fastdds_topic.empty()
                                  ? FLAGS_ros2_topic
                                  : FLAGS_fastdds_topic;
  adapter_config.sample_rate_hz = FLAGS_sample_rate_hz;
  adapter_config.max_pending_samples =
      static_cast<size_t>(FLAGS_max_pending_samples);
  auto adapter = MakeRobotStateAdapter(std::move(adapter_config),
                                       &transport_error_message);
  if (adapter == nullptr || !adapter->Start(&transport_error_message)) {
    std::fprintf(stderr, "%s\n", transport_error_message.c_str());
    return 1;
  }

  glfwSetErrorCallback([](int error, const char* description) {
    std::fprintf(stderr, "GLFW %d: %s\n", error, description);
  });
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_SAMPLES, 4);
  GLFWwindow* window =
      glfwCreateWindow(1280, 760, "Simulation GUI", nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glEnable(GL_MULTISAMPLE);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImPlot::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  const auto* renderer_bytes = glGetString(GL_RENDERER);
  const std::string renderer =
      renderer_bytes == nullptr
          ? "unknown"
          : reinterpret_cast<const char*>(renderer_bytes);
  const std::string transport_label =
      FLAGS_transport == "lcm" ? "LCM" : "ROS 2 / FastDDS";
  SignalWorkspace workspace(
      transport_label, adapter->description(), renderer, FLAGS_max_points,
      light_theme, FLAGS_history_seconds, FLAGS_gap_threshold_ms * 1e-3);

  bool transport_error = false;
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    if (!adapter->Poll(&transport_error_message)) {
      std::fprintf(stderr, "%s\n", transport_error_message.c_str());
      transport_error = true;
      break;
    }
    for (auto& snapshot : adapter->TakeBatch()) {
      workspace.Consume(std::move(snapshot));
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    workspace.Render(adapter.get());

    ImGui::Render();
    int display_width = 0;
    int display_height = 0;
    glfwGetFramebufferSize(window, &display_width, &display_height);
    glViewport(0, 0, display_width, display_height);
    const ImVec4 clear_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return transport_error ? 1 : 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::DoMain();
}
