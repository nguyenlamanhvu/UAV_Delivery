#include "systems/dryden_wind_force_system.h"

#include <drake/multibody/math/spatial_algebra.h>
#include "systems/wind-dynamics/dryden_model.h"

namespace uav_delivery {
namespace systems {

DrydenWindForceSystem::DrydenWindForceSystem(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& target_body_name)
    : plant_(plant), target_body_name_(target_body_name) {
  
  wind_parameters_input_port_index_ = this->DeclareAbstractInputPort(
          "wind_parameters", 
          drake::Value<uav_delivery::lcmt_wind_parameters>())
      .get_index();

  spatial_forces_output_port_index_ = this->DeclareAbstractOutputPort(
          "spatial_forces",
          &DrydenWindForceSystem::CalcSpatialForces)
      .get_index();
}

void DrydenWindForceSystem::CalcSpatialForces(
    const drake::systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {
  
  output->clear();
  
  drake::multibody::ExternallyAppliedSpatialForce<double> wind_force;
  
  try {
      wind_force.body_index = plant_.GetBodyByName(target_body_name_).index();
  } catch (const std::exception& e) {
      return;
  }
  
  wind_force.p_BoBq_B = Eigen::Vector3d::Zero();
  
  const auto& wind_params = get_wind_parameters_input_port().Eval<uav_delivery::lcmt_wind_parameters>(context);
  
  // Safe default for uninitialized LCM message or zero drag
  if (std::isnan(wind_params.drag_coeff) || wind_params.drag_coeff < 1e-6) {
      wind_force.F_Bq_W = drake::multibody::SpatialForce<double>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      output->push_back(wind_force);
      return;
  }
  
  // Calculate Dryden wind gusts using the goromal library
  static dryden_model::DrydenWind wind_model;
  
  double alt = std::isnan(wind_params.altitude) ? 1.0 : std::max(0.1, wind_params.altitude);
  double sig_x = std::isnan(wind_params.gust_sigma[0]) ? 0.0 : wind_params.gust_sigma[0];
  double sig_y = std::isnan(wind_params.gust_sigma[1]) ? 0.0 : wind_params.gust_sigma[1];
  double sig_z = std::isnan(wind_params.gust_sigma[2]) ? 0.0 : wind_params.gust_sigma[2];
  
  wind_model.initialize(wind_params.nominal_wind[0], wind_params.nominal_wind[1], wind_params.nominal_wind[2],
                        sig_x, sig_y, sig_z, alt);
  
  // Hardcode dt as 0.001 for the gust integration step
  Eigen::Vector3d gust_velocity = wind_model.getWind(0.001);
  
  // Limit max gust velocity to avoid explosion
  if (gust_velocity.norm() > 20.0) {
      gust_velocity = gust_velocity.normalized() * 20.0;
  }
  
  double linear_drag_coeff = std::min(wind_params.drag_coeff, 2.0);
  Eigen::Vector3d force_W = linear_drag_coeff * gust_velocity;
  
  // Absolute force clip (e.g. max 50N)
  if (force_W.norm() > 50.0) {
      force_W = force_W.normalized() * 50.0;
  }
  
  Eigen::Vector3d torque_W(0.0, 0.0, 0.0);
  wind_force.F_Bq_W = drake::multibody::SpatialForce<double>(torque_W, force_W);
  output->push_back(wind_force);
}

}  // namespace systems
}  // namespace uav_delivery
