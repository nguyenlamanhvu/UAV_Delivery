#include "gui/signal_workspace.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include "gui/plugins/map_2d_plugin.h"
#include "gui/plugins/meshcat_plugin.h"
#include "gui/plugins/rtsp_plugin.h"
#include <utility>
#include <vector>

#include "imgui.h"
#include "implot.h"

namespace dairlib {
namespace simulation_gui {
namespace {

void ApplyTheme(bool light_theme) {
  if (light_theme) {
    ImGui::StyleColorsLight();
  } else {
    ImGui::StyleColorsDark();
  }
}

ImVec4 ScaleRgb(const ImVec4& color, float scale) {
  return ImVec4(std::min(1.0f, color.x * scale),
                std::min(1.0f, color.y * scale),
                std::min(1.0f, color.z * scale), color.w);
}

bool ActionButton(const char* label, const ImVec4& color, float width) {
  ImGui::PushStyleColor(ImGuiCol_Button, color);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ScaleRgb(color, 1.15f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ScaleRgb(color, 0.82f));
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
  const bool pressed = ImGui::Button(label, ImVec2(width, 30.0f));
  ImGui::PopStyleColor(4);
  return pressed;
}

class PlotBuffer {
 public:
  explicit PlotBuffer(int capacity)
      : time_(std::max(2, capacity)), value_(std::max(2, capacity)) {}

  void Add(double time, double value) {
    time_[offset_] = time;
    value_[offset_] = value;
    offset_ = (offset_ + 1) % capacity();
    count_ = std::min(count_ + 1, capacity());
  }

  void Clear() {
    offset_ = 0;
    count_ = 0;
  }

  int capacity() const { return static_cast<int>(time_.size()); }
  int count() const { return count_; }
  int offset() const { return count_ == capacity() ? offset_ : 0; }
  const double* time_data() const { return time_.data(); }
  const double* value_data() const { return value_.data(); }

  double latest_time() const {
    if (count_ == 0) return 0.0;
    const int index = (offset_ + capacity() - 1) % capacity();
    return time_[index];
  }

 private:
  std::vector<double> time_;
  std::vector<double> value_;
  int offset_{0};
  int count_{0};
};

struct SignalKey {
  std::string source;
  std::string message_type;
  std::string field_path;
  std::string name;

  bool operator==(const SignalKey& other) const {
    return source == other.source && message_type == other.message_type &&
           field_path == other.field_path && name == other.name;
  }
};

std::string CurveLabel(const SignalKey& key) {
  return key.source + " / " + key.field_path + " / " + key.name;
}

struct PlotCurve {
  PlotCurve(SignalKey signal_key, int capacity)
      : key(std::move(signal_key)), label(CurveLabel(key)), plot(capacity) {}

  SignalKey key;
  std::string label;
  PlotBuffer plot;
};

enum class TabType { DATA_PLOT, MAP_2D, MESHCAT, RTSP };

struct PlotTab {
  PlotTab(int tab_id, std::string tab_name)
      : id(tab_id), name(std::move(tab_name)) {}

  int id;
  std::string name;
  std::vector<PlotCurve> curves;
  bool paused{false};
  bool follow_latest{true};
  double history_seconds{10.0};
  int64_t gap_count{0};
  TabType type{TabType::DATA_PLOT};
  std::unique_ptr<GuiPlugin> plugin{nullptr};
};

struct BrowserLeaf {
  SignalKey key;
  double value{0.0};
};

struct BrowserNode {
  explicit BrowserNode(std::string node_label)
      : label(std::move(node_label)) {}

  std::string label;
  std::vector<BrowserNode> children;
  std::vector<BrowserLeaf> leaves;
};

BrowserNode* FindOrAddChild(BrowserNode* parent, const std::string& label) {
  for (BrowserNode& child : parent->children) {
    if (child.label == label) return &child;
  }
  parent->children.emplace_back(label);
  return &parent->children.back();
}

std::vector<std::string> SplitPath(const std::string& path) {
  std::vector<std::string> result;
  size_t begin = 0;
  while (begin < path.size()) {
    const size_t slash = path.find('/', begin);
    const size_t end = slash == std::string::npos ? path.size() : slash;
    if (end > begin) result.push_back(path.substr(begin, end - begin));
    if (slash == std::string::npos) break;
    begin = slash + 1;
  }
  return result;
}

}  // namespace

class SignalWorkspace::Impl {
 public:
  Impl(std::string transport_label, std::string source_description,
       std::string renderer, int max_points, bool light_theme,
       double history_seconds, double gap_threshold_seconds)
      : transport_label_(std::move(transport_label)),
        source_description_(std::move(source_description)),
        renderer_(std::move(renderer)),
        max_points_(std::max(2, max_points)),
        light_theme_(light_theme),
        default_history_seconds_(std::max(0.1, history_seconds)),
        gap_threshold_seconds_(std::max(0.001, gap_threshold_seconds)) {
    AddPlotTab();
    ApplyTheme(light_theme_);
  }

  void Consume(RobotStateSnapshot snapshot) {
    latest_ = std::move(snapshot);
    has_message_ = true;
    ++message_count_;
    last_receive_time_ = std::chrono::steady_clock::now();

    const double time = static_cast<double>(latest_.timestamp_us) * 1e-6;
    for (PlotTab& tab : tabs_) {
      if (tab.plugin) {
        tab.plugin->Consume(snapshot);
      }
      if (tab.paused) continue;
      bool saw_gap = false;
      for (PlotCurve& curve : tab.curves) {
        const double* value = FindValue(curve.key);
        if (value == nullptr) continue;

        if (curve.plot.count() > 0 && time < curve.plot.latest_time()) {
          curve.plot.Clear();
        }
        if (curve.plot.count() > 0 && time <= curve.plot.latest_time()) {
          continue;
        }
        if (curve.plot.count() > 0 &&
            time - curve.plot.latest_time() > gap_threshold_seconds_) {
          curve.plot.Add((curve.plot.latest_time() + time) * 0.5,
                         std::numeric_limits<double>::quiet_NaN());
          saw_gap = true;
        }
        curve.plot.Add(time, *value);
      }
      if (saw_gap) ++tab.gap_count;
    }
  }

  void Render(RobotStateAdapter* adapter) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin("Simulation GUI", nullptr, flags);
    ImGui::DockSpace(ImGui::GetID("MyDockSpace"));

    ImGui::TextDisabled("OpenGL3 renderer: %s", renderer_.c_str());
    RenderStatus();
    RenderTabBar(adapter);
    RenderSourceSelector(adapter);
    ImGui::Separator();

    PlotTab* active_tab = ActiveTab();
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
    const ImVec2 splitter_min = ImGui::GetItemRectMin();
    const ImVec2 splitter_max = ImGui::GetItemRectMax();
    const ImU32 splitter_color = ImGui::GetColorU32(
        splitter_active
            ? ImGuiCol_SeparatorActive
            : (splitter_hovered ? ImGuiCol_SeparatorHovered
                                : ImGuiCol_Separator));
    ImGui::GetWindowDrawList()->AddLine(
        ImVec2((splitter_min.x + splitter_max.x) * 0.5f, splitter_min.y),
        ImVec2((splitter_min.x + splitter_max.x) * 0.5f, splitter_max.y),
        splitter_color, splitter_active ? 3.0f : 1.0f);
    if (splitter_hovered) {
      ImGui::SetTooltip(
          "Drag to resize the signal browser\nDouble-click to reset");
    }

    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginChild("plot_workspace", ImVec2(0, available.y),
                      ImGuiChildFlags_Borders);
    if (active_tab != nullptr) RenderPlot(active_tab);
    ImGui::EndChild();
    ImGui::End();
  }

 private:
  void RenderStatus() const {
    if (!has_message_) {
      ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Waiting for %s",
                         source_description_.c_str());
      return;
    }

    const double age = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - last_receive_time_)
                           .count();
    const bool receiving = age < 1.0;
    ImGui::TextColored(receiving ? ImVec4(0.2f, 0.75f, 0.25f, 1.0f)
                                 : ImVec4(1.0f, 0.25f, 0.2f, 1.0f),
                       "%s", receiving ? "Receiving" : "Stale");
    ImGui::SameLine();
    ImGui::Text("source=%s  displayed=%lld  age=%.3f s  utime=%lld",
                source_description_.c_str(),
                static_cast<long long>(message_count_), age,
                static_cast<long long>(latest_.timestamp_us));

    size_t signal_count = 0;
    for (const RobotStateSignalGroup& group : latest_.signal_groups) {
      signal_count += std::min(group.names.size(), group.values.size());
    }
    ImGui::Text("groups=%zu  numeric signals=%zu", latest_.signal_groups.size(),
                signal_count);
  }

  void AddPlotTab() {
    const int id = next_tab_id_++;
    PlotTab tab(id, "Plot " + std::to_string(id));
    tab.history_seconds = default_history_seconds_;
    tabs_.push_back(std::move(tab));
    active_tab_id_ = id;
    requested_tab_id_ = id;
  }

  void RenderTabBar(RobotStateAdapter* adapter) {
    bool add_tab = false;
    bool open_source_selector = false;
    if (ImGui::Button("+ Add Module Window")) add_tab = true;
    ImGui::SameLine();
    if (ImGui::Button("Select Sources...")) open_source_selector = true;
    if (add_tab) AddPlotTab();
    if (open_source_selector) OpenSourceSelector(adapter);
    return;
  }
  // Old implementation ignored
  void RenderTabBar_Old(RobotStateAdapter* adapter) {
    int close_index = -1;
    bool add_tab = false;
    bool open_source_selector = false;
    const int requested_tab_id = requested_tab_id_;
    requested_tab_id_ = -1;

    if (ImGui::BeginTabBar("plot_tabs",
                           ImGuiTabBarFlags_Reorderable |
                               ImGuiTabBarFlags_AutoSelectNewTabs)) {
      for (int i = 0; i < static_cast<int>(tabs_.size()); ++i) {
        PlotTab& tab = tabs_[i];
        bool open = true;
        const std::string label =
            tab.name + " (" + std::to_string(tab.curves.size()) +
            ")###plot_tab_" + std::to_string(tab.id);
        const ImGuiTabItemFlags flags =
            tab.id == requested_tab_id ? ImGuiTabItemFlags_SetSelected
                                       : ImGuiTabItemFlags_None;
        bool* open_pointer = tabs_.size() > 1 ? &open : nullptr;
        if (ImGui::BeginTabItem(label.c_str(), open_pointer, flags)) {
          active_tab_id_ = tab.id;
          ImGui::EndTabItem();
        }
        if (!open) close_index = i;
      }
      if (ImGui::TabItemButton("+ Add plot", ImGuiTabItemFlags_Trailing)) {
        add_tab = true;
      }
      if (ImGui::TabItemButton("Select sources...",
                               ImGuiTabItemFlags_Trailing)) {
        open_source_selector = true;
      }
      ImGui::EndTabBar();
    }

    if (close_index >= 0) {
      const int closed_id = tabs_[close_index].id;
      tabs_.erase(tabs_.begin() + close_index);
      if (active_tab_id_ == closed_id && !tabs_.empty()) {
        const int next = std::min(close_index,
                                  static_cast<int>(tabs_.size()) - 1);
        active_tab_id_ = tabs_[next].id;
        requested_tab_id_ = active_tab_id_;
      }
    }
    if (add_tab) AddPlotTab();
    if (open_source_selector) OpenSourceSelector(adapter);
  }

  void OpenSourceSelector(RobotStateAdapter* adapter) {
    pending_source_selection_.clear();
    source_selection_error_.clear();
    if (adapter != nullptr) {
      for (const RobotStateSourceInfo& source : adapter->sources()) {
        pending_source_selection_[source.id] = source.selected;
      }
    }
    ImGui::OpenPopup("Select middleware sources");
  }

  void RenderSourceSelector(RobotStateAdapter* adapter) {
    ImGui::SetNextWindowSize(ImVec2(820.0f, 520.0f),
                             ImGuiCond_FirstUseEver);
    if (!ImGui::BeginPopupModal("Select middleware sources", nullptr)) return;

    ImGui::TextUnformatted(
        "Select one or more topics/channels to subscribe and plot.");
    ImGui::TextDisabled(
        "Unsupported datatypes remain visible until a decoder is registered.");
    source_filter_.Draw("Filter topics or datatypes", -1.0f);

    const std::vector<RobotStateSourceInfo> sources =
        adapter == nullptr ? std::vector<RobotStateSourceInfo>{}
                           : adapter->sources();
    for (const RobotStateSourceInfo& source : sources) {
      pending_source_selection_.try_emplace(source.id, source.selected);
    }

    if (ImGui::Button("Select all supported")) {
      for (const RobotStateSourceInfo& source : sources) {
        if (source.supported) pending_source_selection_[source.id] = true;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear selection")) {
      for (const RobotStateSourceInfo& source : sources) {
        pending_source_selection_[source.id] = false;
      }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu discovered", sources.size());

    const float footer_height = ImGui::GetFrameHeightWithSpacing() * 2.5f;
    ImGui::BeginChild("source_table_region", ImVec2(0, -footer_height),
                      ImGuiChildFlags_Borders);
    if (ImGui::BeginTable(
            "source_table", 3,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY |
                ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 34.0f);
      ImGui::TableSetupColumn("Topic / channel",
                              ImGuiTableColumnFlags_WidthStretch, 0.48f);
      ImGui::TableSetupColumn("Datatype",
                              ImGuiTableColumnFlags_WidthStretch, 0.52f);
      ImGui::TableHeadersRow();

      for (const RobotStateSourceInfo& source : sources) {
        const std::string searchable =
            source.display_name + " " + source.message_type;
        if (!source_filter_.PassFilter(searchable.c_str())) continue;

        ImGui::PushID(source.id.c_str());
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        bool selected = pending_source_selection_[source.id];
        if (!source.supported) ImGui::BeginDisabled();
        if (ImGui::Checkbox("##selected", &selected)) {
          pending_source_selection_[source.id] = selected;
        }
        if (!source.supported) ImGui::EndDisabled();

        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(source.display_name.c_str());
        ImGui::TableSetColumnIndex(2);
        if (source.supported) {
          ImGui::TextUnformatted(source.message_type.c_str());
        } else {
          ImGui::TextDisabled("%s  (decoder unavailable)",
                              source.message_type.c_str());
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    ImGui::EndChild();

    if (!source_selection_error_.empty()) {
      ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "%s",
                         source_selection_error_.c_str());
    }
    if (ImGui::Button("Apply", ImVec2(100.0f, 0.0f))) {
      bool success = adapter != nullptr;
      source_selection_error_.clear();
      if (adapter == nullptr) {
        source_selection_error_ = "Middleware adapter is unavailable";
      } else {
        for (const RobotStateSourceInfo& source : sources) {
          const bool selected = pending_source_selection_[source.id];
          if (selected == source.selected) continue;
          if (!adapter->SetSourceSelected(source.id, selected,
                                          &source_selection_error_)) {
            success = false;
            break;
          }
        }
      }
      if (success) ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  PlotTab* ActiveTab() {
    for (PlotTab& tab : tabs_) {
      if (tab.id == active_tab_id_) return &tab;
    }
    return tabs_.empty() ? nullptr : &tabs_.front();
  }

  BrowserNode BuildBrowserTree() const {
    BrowserNode root(transport_label_);
    for (const RobotStateSignalGroup& group : latest_.signal_groups) {
      BrowserNode* node = FindOrAddChild(&root, group.source);
      node = FindOrAddChild(node, group.message_type);
      for (const std::string& segment : SplitPath(group.field_path)) {
        node = FindOrAddChild(node, segment);
      }

      const size_t count = std::min(group.names.size(), group.values.size());
      for (size_t i = 0; i < count; ++i) {
        node->leaves.push_back(BrowserLeaf{
            SignalKey{group.source, group.message_type, group.field_path,
                      group.names[i]},
            group.values[i]});
      }
    }
    return root;
  }

  bool HasCurve(const PlotTab& tab, const SignalKey& key) const {
    return std::any_of(tab.curves.begin(), tab.curves.end(),
                       [&key](const PlotCurve& curve) {
                         return curve.key == key;
                       });
  }

  void SetCurveSelected(PlotTab* tab, const BrowserLeaf& leaf, bool selected) {
    const auto found = std::find_if(
        tab->curves.begin(), tab->curves.end(),
        [&leaf](const PlotCurve& curve) { return curve.key == leaf.key; });
    if (!selected) {
      if (found != tab->curves.end()) tab->curves.erase(found);
      return;
    }
    if (found != tab->curves.end()) return;

    if (tab->curves.empty()) {
      const size_t slash = leaf.key.field_path.find_last_of('/');
      tab->name = slash == std::string::npos
                      ? leaf.key.field_path
                      : leaf.key.field_path.substr(slash + 1);
    }
    tab->curves.emplace_back(leaf.key, max_points_);
    if (has_message_) {
      tab->curves.back().plot.Add(
          static_cast<double>(latest_.timestamp_us) * 1e-6, leaf.value);
    }
  }

  void RenderBrowserNode(const BrowserNode& node, PlotTab* tab, int depth) {
    ImGui::PushID(node.label.c_str());
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanAvailWidth;
    if (depth < 3) flags |= ImGuiTreeNodeFlags_DefaultOpen;
    const bool open =
        ImGui::TreeNodeEx("##node", flags, "%s", node.label.c_str());
    if (open) {
      for (const BrowserNode& child : node.children) {
        RenderBrowserNode(child, tab, depth + 1);
      }

      if (!node.leaves.empty() && tab != nullptr) {
        const bool all_selected =
            std::all_of(node.leaves.begin(), node.leaves.end(),
                        [this, tab](const BrowserLeaf& leaf) {
                          return HasCurve(*tab, leaf.key);
                        });
        if (ImGui::SmallButton(all_selected ? "Remove all" : "Add all")) {
          for (const BrowserLeaf& leaf : node.leaves) {
            SetCurveSelected(tab, leaf, !all_selected);
          }
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%zu signals", node.leaves.size());

        for (int i = 0; i < static_cast<int>(node.leaves.size()); ++i) {
          const BrowserLeaf& leaf = node.leaves[i];
          const bool selected = HasCurve(*tab, leaf.key);
          char value[64];
          std::snprintf(value, sizeof(value), "%.6g", leaf.value);
          const std::string row =
              leaf.key.name + "    " + value + "###signal";
          ImGui::PushID(i);
          if (ImGui::Selectable(row.c_str(), selected)) {
            SetCurveSelected(tab, leaf, !selected);
          }
          if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "%s\n%s\nClick to %s this signal in %s",
                leaf.key.field_path.c_str(), leaf.key.source.c_str(),
                selected ? "remove" : "plot", tab->name.c_str());
          }
          ImGui::PopID();
        }
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
  }

  void RenderSignalBrowser(PlotTab* tab) {
    if (tab != nullptr) {
      const char* types[] = { "Data Plot", "2D Map", "MeshCat Control", "RTSP Camera" };
      int current_type = static_cast<int>(tab->type);
      if (ImGui::Combo("Plugin Type", &current_type, types, 4)) {
        tab->type = static_cast<TabType>(current_type);
        if (tab->type == TabType::MAP_2D) tab->plugin = std::make_unique<Map2DPlugin>();
        else if (tab->type == TabType::MESHCAT) tab->plugin = std::make_unique<MeshcatPlugin>();
        else if (tab->type == TabType::RTSP) tab->plugin = std::make_unique<RtspPlugin>();
        else if (tab->type == TabType::DOCS) tab->plugin = std::make_unique<DocsPlugin>();
        else tab->plugin.reset();
      }
      ImGui::Spacing();
      if (tab->type == TabType::DATA_PLOT) {
        RenderPlotConfiguration(tab);
        ImGui::Spacing();
      }
    }
    if (tab == nullptr || tab->type != TabType::DATA_PLOT) return;
    ImGui::TextUnformatted("Signals");
    if (tab != nullptr) {
      ImGui::SameLine();
      ImGui::TextDisabled("click to add/remove from %s", tab->name.c_str());
    }
    ImGui::Separator();
    if (!has_message_) {
      ImGui::TextDisabled("Waiting for the signal schema...");
      return;
    }
    const BrowserNode root = BuildBrowserTree();
    RenderBrowserNode(root, tab, 0);
  }

  const double* FindValue(const SignalKey& key) const {
    for (const RobotStateSignalGroup& group : latest_.signal_groups) {
      if (group.source != key.source || group.message_type != key.message_type ||
          group.field_path != key.field_path) {
        continue;
      }
      const auto name =
          std::find(group.names.begin(), group.names.end(), key.name);
      if (name == group.names.end()) return nullptr;
      const size_t index = static_cast<size_t>(name - group.names.begin());
      return index < group.values.size() ? &group.values[index] : nullptr;
    }
    return nullptr;
  }

  void ClearHistory(PlotTab* tab) {
    for (PlotCurve& curve : tab->curves) curve.plot.Clear();
    tab->gap_count = 0;
  }

  void RenderPlotConfiguration(PlotTab* tab) {
    int sample_count = 0;
    for (const PlotCurve& curve : tab->curves) {
      sample_count = std::max(sample_count, curve.plot.count());
    }
    const bool compact = ImGui::GetContentRegionAvail().x < 560.0f;
    const ImVec4 card_color =
        light_theme_ ? ImVec4(0.96f, 0.97f, 0.985f, 1.0f)
                     : ImVec4(0.09f, 0.105f, 0.125f, 1.0f);
    bool request_remove = false;

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.0f);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, card_color);
    ImGui::BeginChild(
        "plot_configuration", ImVec2(0, 0),
        ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding |
            ImGuiChildFlags_AutoResizeY);

    ImGui::TextDisabled(compact ? "PLOT CONFIG" : "PLOT CONFIGURATION");
    ImGui::SameLine();
    ImGui::Text("%s  |  %zu curves", tab->name.c_str(), tab->curves.size());

    const int columns = compact ? 2 : 4;
    if (ImGui::BeginTable(
            "plot_action_buttons", columns,
            ImGuiTableFlags_SizingStretchSame |
                ImGuiTableFlags_NoSavedSettings)) {
      ImGui::TableNextColumn();
      const ImVec4 pause_color =
          tab->paused ? ImVec4(0.88f, 0.48f, 0.10f, 1.0f)
                      : ImVec4(0.25f, 0.42f, 0.60f, 1.0f);
      if (ActionButton(tab->paused ? "Resume plotting##pause"
                                   : "Pause plotting##pause",
                       pause_color, ImGui::GetContentRegionAvail().x)) {
        tab->paused = !tab->paused;
      }

      ImGui::TableNextColumn();
      const ImVec4 follow_color =
          tab->follow_latest ? ImVec4(0.14f, 0.58f, 0.36f, 1.0f)
                             : ImVec4(0.34f, 0.39f, 0.46f, 1.0f);
      if (ActionButton(tab->follow_latest
                           ? "Following latest##follow"
                           : "Follow latest##follow",
                       follow_color, ImGui::GetContentRegionAvail().x)) {
        tab->follow_latest = !tab->follow_latest;
      }

      ImGui::TableNextColumn();
      if (tab->curves.empty()) ImGui::BeginDisabled();
      if (ActionButton("Clear history##clear",
                       ImVec4(0.18f, 0.48f, 0.72f, 1.0f),
                       ImGui::GetContentRegionAvail().x)) {
        ClearHistory(tab);
      }
      if (tab->curves.empty()) ImGui::EndDisabled();

      ImGui::TableNextColumn();
      if (tab->curves.empty()) ImGui::BeginDisabled();
      if (ActionButton("Remove all curves##remove",
                       ImVec4(0.78f, 0.18f, 0.20f, 1.0f),
                       ImGui::GetContentRegionAvail().x)) {
        request_remove = true;
      }
      if (tab->curves.empty()) ImGui::EndDisabled();
      ImGui::EndTable();
    }

    const ImVec4 theme_color =
        light_theme_ ? ImVec4(0.42f, 0.38f, 0.62f, 1.0f)
                     : ImVec4(0.28f, 0.32f, 0.42f, 1.0f);
    const float theme_button_width = compact ? 104.0f : 112.0f;
    if (ActionButton(light_theme_ ? "Theme: Light##theme"
                                  : "Theme: Dark##theme",
                     theme_color, theme_button_width)) {
      light_theme_ = !light_theme_;
      ApplyTheme(light_theme_);
    }
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled(compact ? "%d/%d | gaps %lld"
                                : "Samples %d / %d  |  Gaps %lld",
                        sample_count, max_points_,
                        static_cast<long long>(tab->gap_count));

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(compact ? "History size" : "Visible history size");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(compact ? 82.0f : 100.0f);
    ImGui::InputDouble("s##history_window", &tab->history_seconds, 1.0, 10.0,
                       "%.1f");
    tab->history_seconds = std::max(0.1, tab->history_seconds);

    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);

    if (request_remove) ImGui::OpenPopup("Remove all curves?##confirm_remove");
    if (ImGui::BeginPopupModal("Remove all curves?##confirm_remove", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("Remove all %zu curves from %s?", tab->curves.size(),
                  tab->name.c_str());
      ImGui::TextDisabled(
          "Only this tab is affected. Other plot tabs keep their curves.");
      ImGui::Separator();
      if (ActionButton("Remove curves##confirm",
                       ImVec4(0.78f, 0.18f, 0.20f, 1.0f), 130.0f)) {
        tab->curves.clear();
        tab->gap_count = 0;
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine();
      if (ActionButton("Cancel##cancel",
                       ImVec4(0.34f, 0.39f, 0.46f, 1.0f), 90.0f)) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
  }

  void RenderPlot(PlotTab* tab) {
    bool has_plot_time = false;
    double latest_time = 0.0;
    for (const PlotCurve& curve : tab->curves) {
      if (curve.plot.count() > 0) {
        latest_time = std::max(latest_time, curve.plot.latest_time());
        has_plot_time = true;
      }
    }
    if (tab->curves.empty()) {
      ImGui::TextDisabled(
          "Choose signals from the hierarchy on the left, or use Add all.");
      return;
    }

    // Group curves by their field_path (e.g., "position", "velocity") to create separate subplots
    std::vector<std::string> groups;
    for (const PlotCurve& curve : tab->curves) {
      if (std::find(groups.begin(), groups.end(), curve.key.field_path) == groups.end()) {
        groups.push_back(curve.key.field_path);
      }
    }

    int num_plots = groups.size();
    int cols = std::ceil(std::sqrt(num_plots));
    int rows = std::ceil((float)num_plots / cols);

    if (ImPlot::BeginSubplots("##signal_grid", rows, cols, ImVec2(-1, -1))) {
      for (const std::string& group : groups) {
        if (ImPlot::BeginPlot(group.c_str())) {
          ImPlot::SetupAxes("simulation time (s)", "value", ImPlotAxisFlags_None,
                            ImPlotAxisFlags_AutoFit);
          if (tab->follow_latest && has_plot_time) {
            ImPlot::SetupAxisLimits(ImAxis_X1,
                                    latest_time - tab->history_seconds, latest_time,
                                    ImGuiCond_Always);
          }
          for (const PlotCurve& curve : tab->curves) {
            if (curve.key.field_path != group || curve.plot.count() == 0) continue;
            ImPlot::PlotLine(curve.label.c_str(), curve.plot.time_data(),
                             curve.plot.value_data(), curve.plot.count(), 0,
                             curve.plot.offset(), sizeof(double));
          }
          ImPlot::EndPlot();
        }
      }
      ImPlot::EndSubplots();
    }
  }

  std::string transport_label_;
  std::string source_description_;
  std::string renderer_;
  int max_points_;
  bool light_theme_;
  double default_history_seconds_;
  double gap_threshold_seconds_;
  RobotStateSnapshot latest_;
  bool has_message_{false};
  int64_t message_count_{0};
  std::chrono::steady_clock::time_point last_receive_time_{};
  std::vector<PlotTab> tabs_;
  int next_tab_id_{1};
  int active_tab_id_{-1};
  int requested_tab_id_{-1};
  float browser_width_{280.0f};
  ImGuiTextFilter source_filter_;
  std::unordered_map<std::string, bool> pending_source_selection_;
  std::string source_selection_error_;
};

SignalWorkspace::SignalWorkspace(
    std::string transport_label, std::string source_description,
    std::string renderer, int max_points, bool light_theme,
    double history_seconds, double gap_threshold_seconds)
    : impl_(std::make_unique<Impl>(
          std::move(transport_label), std::move(source_description),
          std::move(renderer), max_points, light_theme, history_seconds,
          gap_threshold_seconds)) {}

SignalWorkspace::~SignalWorkspace() = default;

void SignalWorkspace::Consume(RobotStateSnapshot snapshot) {
  impl_->Consume(std::move(snapshot));
}

void SignalWorkspace::Render(RobotStateAdapter* adapter) {
  impl_->Render(adapter);
}

}  // namespace simulation_gui
}  // namespace dairlib
