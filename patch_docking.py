import re

with open("gui/signal_workspace.cc", "r") as f:
    content = f.read()

# Add DockSpace to the main window
content = content.replace("""    ImGui::Begin("Simulation GUI", nullptr, flags);

    ImGui::TextDisabled("OpenGL3 renderer: %s", renderer_.c_str());""",
"""    ImGui::Begin("Simulation GUI", nullptr, flags);
    ImGui::DockSpace(ImGui::GetID("MyDockSpace"));

    ImGui::TextDisabled("OpenGL3 renderer: %s", renderer_.c_str());""")


# Change TabBar to just render Windows
content = content.replace("""    PlotTab* active_tab = ActiveTab();
    const float splitter_width = 8.0f;
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const float minimum_browser_width = std::min(180.0f, available.x * 0.4f);
    const float maximum_browser_width =
        std::max(minimum_browser_width,
                 available.x - splitter_width - 320.0f);
    browser_width_ = std::clamp(browser_width_, minimum_browser_width,
                                maximum_browser_width);

    ImGui::BeginChild("signal_browser",
                      ImVec2(browser_width_, available.y),
                      ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);
    RenderSignalBrowser(active_tab);
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    ImGui::InvisibleButton("##signal_browser_splitter",
                           ImVec2(splitter_width, available.y));
    const bool splitter_hovered = ImGui::IsItemHovered();
    const bool splitter_active = ImGui::IsItemActive();
    if (splitter_hovered || splitter_active) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }
    if (splitter_active) {
      browser_width_ += ImGui::GetIO().MouseDelta.x;
      browser_width_ = std::clamp(browser_width_, minimum_browser_width,
                                  maximum_browser_width);
    }
    if (splitter_hovered &&
        ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      browser_width_ = 280.0f;
    }
    ImGui::SameLine(0.0f, 0.0f);

    ImGui::BeginChild("signal_plot", ImGui::GetContentRegionAvail());
    if (active_tab != nullptr) RenderPlot(active_tab);
    ImGui::EndChild();

    ImGui::End();""",
"""    // The main window just holds the dockspace and the sidebar
    ImGui::End();

    // Render the Signal Browser in its own Dockable Window
    ImGui::Begin("Signal Browser");
    PlotTab* active_tab = ActiveTab();
    RenderSignalBrowser(active_tab);
    ImGui::End();

    // Render each Tab as a separate Dockable Window
    for (PlotTab& tab : tabs_) {
        bool open = true;
        ImGui::Begin((tab.name + "###" + std::to_string(tab.id)).c_str(), &open);
        if (ImGui::IsWindowFocused()) {
            active_tab_id_ = tab.id;
        }
        RenderPlot(&tab);
        ImGui::End();
        if (!open) {
            // Mark for deletion if closed
            tab.id = -1;
        }
    }

    // Cleanup closed tabs
    tabs_.erase(std::remove_if(tabs_.begin(), tabs_.end(), [](const PlotTab& t) { return t.id == -1; }), tabs_.end());
""")

# Simplify RenderTabBar since we don't need a real tab bar anymore
content = content.replace("""  void RenderTabBar(RobotStateAdapter* adapter) {""",
"""  void RenderTabBar(RobotStateAdapter* adapter) {
    bool add_tab = false;
    bool open_source_selector = false;
    if (ImGui::Button("+ Add Module Window")) add_tab = true;
    ImGui::SameLine();
    if (ImGui::Button("Select Sources...")) open_source_selector = true;
    if (add_tab) AddPlotTab();
    if (open_source_selector) OpenSourceSelector(adapter);
    return;
  // Old implementation ignored
  void RenderTabBar_Old(RobotStateAdapter* adapter) {""")

with open("gui/signal_workspace.cc", "w") as f:
    f.write(content)
