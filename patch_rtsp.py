import re

with open("gui/signal_workspace.cc", "r") as f:
    content = f.read()

# Add include
content = content.replace('#include "gui/plugins/meshcat_plugin.h"', 
'#include "gui/plugins/meshcat_plugin.h"\n#include "gui/plugins/rtsp_plugin.h"')

# Add enum
content = content.replace("enum class TabType { DATA_PLOT, MAP_2D, MESHCAT };",
"enum class TabType { DATA_PLOT, MAP_2D, MESHCAT, RTSP };")

# Add to combo
content = content.replace('const char* types[] = { "Data Plot", "2D Map", "MeshCat Control" };',
'const char* types[] = { "Data Plot", "2D Map", "MeshCat Control", "RTSP Camera" };')

# Add to type change logic
content = content.replace("""        else if (tab->type == TabType::MESHCAT) tab->plugin = std::make_unique<MeshcatPlugin>();""",
"""        else if (tab->type == TabType::MESHCAT) tab->plugin = std::make_unique<MeshcatPlugin>();
        else if (tab->type == TabType::RTSP) tab->plugin = std::make_unique<RtspPlugin>();""")

# Update Combo length
content = content.replace('if (ImGui::Combo("Plugin Type", &current_type, types, 3))',
'if (ImGui::Combo("Plugin Type", &current_type, types, 4))')

with open("gui/signal_workspace.cc", "w") as f:
    f.write(content)
