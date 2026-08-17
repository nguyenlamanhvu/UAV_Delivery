with open("BUILD.bazel", "r") as f:
    content = f.read()

# I replaced systems/se3_controller.cc with systems/nmpc_controller.cc in a previous task
content = content.replace('"systems/nmpc_controller.cc"', '"systems/se3_controller.cc"')
content = content.replace('"systems/nmpc_controller.h"', '"systems/se3_controller.h"')

with open("BUILD.bazel", "w") as f:
    f.write(content)
