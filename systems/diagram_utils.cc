#include "systems/diagram_utils.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace uav_delivery {
namespace systems {
namespace {

std::string ShellQuote(const std::string& value) {
  std::string quoted = "'";
  for (const char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

}  // namespace

void MaybeWriteDiagramSvg(const drake::systems::Diagram<double>& diagram,
                          const std::string& svg_path,
                          const std::string& binary_name) {
  const std::filesystem::path binary_path(binary_name);
  const std::string default_file =
      binary_path.filename().empty() ? "diagram.svg"
                                     : binary_path.filename().string() + ".svg";

  std::filesystem::path output = svg_path.empty() ? std::filesystem::path(default_file)
                                                  : std::filesystem::path(svg_path);
  if (std::filesystem::is_directory(output)) {
    output /= default_file;
  } else if (output.extension() != ".svg") {
    output += ".svg";
  }
  const std::filesystem::path dot_path = output.string() + ".dot";
  {
    std::ofstream dot(dot_path);
    dot << diagram.GetGraphvizString();
  }

  const std::string command =
      "dot -Tsvg " + ShellQuote(dot_path.string()) + " -o " +
      ShellQuote(output.string());
  const int rc = std::system(command.c_str());
  if (rc == 0) {
    std::cout << "Wrote diagram SVG: " << output << "\n";
  } else {
    std::cout << "Could not run Graphviz dot. Wrote diagram DOT: " << dot_path
              << "\n";
  }
}

}  // namespace systems
}  // namespace uav_delivery
