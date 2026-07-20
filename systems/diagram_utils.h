#pragma once

#include <string>

#include "drake/systems/framework/diagram.h"

namespace uav_delivery {
namespace systems {

void MaybeWriteDiagramSvg(const drake::systems::Diagram<double>& diagram,
                          const std::string& svg_path,
                          const std::string& binary_name);

}  // namespace systems
}  // namespace uav_delivery
