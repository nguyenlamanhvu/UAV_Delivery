#pragma once

#include <string>

#include "drake/systems/framework/diagram.h"

namespace uav_delivery {
namespace systems {

std::string ActorMeshcatPath(const std::string& actor_name);

void MaybeWriteDiagramSvg(const drake::systems::Diagram<double>& diagram,
                          const std::string& svg_path);

}  // namespace systems
}  // namespace uav_delivery
