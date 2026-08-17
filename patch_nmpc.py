import re
with open("systems/nmpc_controller.cc", "r") as f:
    content = f.read()

debug_code = """
  static int count = 0;
  if (count++ < 10) {
      std::cout << "[DEBUG] x0: " << x0.transpose() << std::endl;
      std::cout << "[DEBUG] x_ref: " << x_ref.transpose() << std::endl;
      std::cout << "[DEBUG] u_opt: " << u_opt.transpose() << std::endl;
      std::cout << "[DEBUG] u_traj_[0]: " << u_traj_[0].transpose() << std::endl;
  }
"""

content = content.replace('output->utime = static_cast<int64_t>', debug_code + '\n  output->utime = static_cast<int64_t>')

with open("systems/nmpc_controller.cc", "w") as f:
    f.write(content)
