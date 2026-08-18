import re

with open("gui/signal_workspace.cc", "r") as f:
    content = f.read()

# Add plugin to PlotTab
content = content.replace("struct PlotTab {\n  PlotTab(int tab_id, std::string tab_name)", 
"""enum class TabType { DATA_PLOT, MAP_2D, MESHCAT };

struct PlotTab {
  PlotTab(int tab_id, std::string tab_name)""")

content = content.replace("  int64_t gap_count{0};\n};", 
"""  int64_t gap_count{0};
  TabType type{TabType::DATA_PLOT};
  std::unique_ptr<GuiPlugin> plugin{nullptr};
};""")

# Add to Consume
content = content.replace("""    const double time = static_cast<double>(latest_.timestamp_us) * 1e-6;
    for (PlotTab& tab : tabs_) {""", 
"""    const double time = static_cast<double>(latest_.timestamp_us) * 1e-6;
    for (PlotTab& tab : tabs_) {
      if (tab.plugin) {
        tab.plugin->Consume(snapshot);
      }""")

# Add Plugin Selector to RenderSignalBrowser
content = content.replace("""  void RenderSignalBrowser(PlotTab* tab) {
    if (tab != nullptr) {
      RenderPlotConfiguration(tab);
      ImGui::Spacing();
    }""", 
"""  void RenderSignalBrowser(PlotTab* tab) {
    if (tab != nullptr) {
      const char* types[] = { "Data Plot", "2D Map", "MeshCat Control" };
      int current_type = static_cast<int>(tab->type);
      if (ImGui::Combo("Plugin Type", &current_type, types, 3)) {
        tab->type = static_cast<TabType>(current_type);
        if (tab->type == TabType::MAP_2D) tab->plugin = std::make_unique<Map2DPlugin>();
        else if (tab->type == TabType::MESHCAT) tab->plugin = std::make_unique<MeshcatPlugin>();
        else tab->plugin.reset();
      }
      ImGui::Spacing();
      if (tab->type == TabType::DATA_PLOT) {
        RenderPlotConfiguration(tab);
        ImGui::Spacing();
      }
    }""")

# Hide Data-specific browser parts when not DataPlot
content = content.replace("""    ImGui::TextUnformatted("Signals");""",
"""    if (tab == nullptr || tab->type != TabType::DATA_PLOT) return;
    ImGui::TextUnformatted("Signals");""")

# Render Plugin in RenderPlot
content = content.replace("""  void RenderPlot(PlotTab* tab) {
    if (tab->curves.empty()) {""",
"""  void RenderPlot(PlotTab* tab) {
    if (tab->plugin) {
      tab->plugin->Render();
      return;
    }
    if (tab->curves.empty()) {""")

with open("gui/signal_workspace.cc", "w") as f:
    f.write(content)
