#include "systems/dryden_wind_force_system.h"

#include <gtest/gtest.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>

namespace uav_delivery {
namespace systems {
namespace {

GTEST_TEST(DrydenWindForceSystemTest, BasicForcesOutput) {
  drake::systems::DiagramBuilder<double> builder;
  drake::multibody::MultibodyPlant<double> plant(0.0);
  
  // Add a dummy body to the plant
  plant.AddRigidBody("drone_base_link", drake::multibody::SpatialInertia<double>::MakeUnitary());
  plant.Finalize();

  auto wind_system = builder.AddSystem<DrydenWindForceSystem>(plant, "drone_base_link");
  
  auto context = wind_system->CreateDefaultContext();
  const auto& output_port = wind_system->get_spatial_forces_output_port();
  
  const auto& spatial_forces = output_port.Eval<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>(*context);
  
  EXPECT_EQ(spatial_forces.size(), 1);
  EXPECT_EQ(spatial_forces[0].body_index, plant.GetBodyByName("drone_base_link").index());
  
  // Gusts are pseudo-random, so we just verify that a force vector is generated
  // and the torque is zero.
  EXPECT_EQ(spatial_forces[0].F_Bq_W.rotational()[0], 0.0);
  EXPECT_EQ(spatial_forces[0].F_Bq_W.rotational()[1], 0.0);
  EXPECT_EQ(spatial_forces[0].F_Bq_W.rotational()[2], 0.0);
}

}  // namespace
}  // namespace systems
}  // namespace uav_delivery
