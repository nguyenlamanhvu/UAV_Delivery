#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "uav_delivery/lcmt_wind_parameters.hpp"

namespace uav_delivery {
namespace systems {

/// A LeafSystem that computes wind disturbance forces (Dryden model) 
/// and applies them to a MultibodyPlant.
class DrydenWindForceSystem : public drake::systems::LeafSystem<double> {
public:
    DrydenWindForceSystem(const drake::multibody::MultibodyPlant<double>& plant, 
                          const std::string& target_body_name);

    const drake::systems::OutputPort<double>& get_spatial_forces_output_port() const {
        return this->get_output_port(spatial_forces_output_port_index_);
    }
    
    const drake::systems::InputPort<double>& get_wind_parameters_input_port() const {
        return this->get_input_port(wind_parameters_input_port_index_);
    }

private:
    void CalcSpatialForces(
        const drake::systems::Context<double>& context,
        std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;

    const drake::multibody::MultibodyPlant<double>& plant_;
    std::string target_body_name_;
    drake::systems::OutputPortIndex spatial_forces_output_port_index_;
    drake::systems::InputPortIndex wind_parameters_input_port_index_;
};

}  // namespace systems
}  // namespace uav_delivery
