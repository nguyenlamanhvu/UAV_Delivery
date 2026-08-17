with open("src/quadrotor_se3_controller.cc", "r") as f:
    content = f.read()

content = content.replace("systems::NmpcController", "systems::Se3Controller")

with open("src/quadrotor_se3_controller.cc", "w") as f:
    f.write(content)
