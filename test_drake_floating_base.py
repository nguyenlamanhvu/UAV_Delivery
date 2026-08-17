import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RigidTransform, RollPitchYaw

plant = MultibodyPlant(0.0)
body = plant.AddRigidBody("body")
plant.Finalize()

context = plant.CreateDefaultContext()
plant.SetFreeBodyPose(context, body, RigidTransform(RollPitchYaw(np.pi/2, 0, 0), [0, 0, 0]))

# Set spatial velocity
V_WB = np.array([0, 0, 0, 1.0, 0, 0]) # vx = 1.0
plant.SetFreeBodySpatialVelocity(body, V_WB, context)

state = context.get_continuous_state_vector().CopyToVector()
print(f"State vector (v part): {state[-6:]}")

