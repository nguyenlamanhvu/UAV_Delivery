with open("src/quadrotor_se3_controller.cc", "r") as f:
    content = f.read()

content = content.replace('#include "systems/nmpc_controller.h"', '#include "systems/se3_controller.h"')
content = content.replace('auto* controller = builder.AddSystem<NmpcController>(params);', 'auto* controller = builder.AddSystem<Se3Controller>(params);')

with open("src/quadrotor_se3_controller.cc", "w") as f:
    f.write(content)
