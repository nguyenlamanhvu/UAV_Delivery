#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm.h"
#include "params/moving_target_params.h"
#include "uav_delivery/lcmt_moving_target_teleop_command.hpp"

DEFINE_string(config, "config/moving_target.yaml",
              "YAML file containing MovingTargetSimParams.");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL for this instance");

namespace uav_delivery {
namespace {

class RawTerminalGuard {
 public:
  RawTerminalGuard() : enabled_(isatty(STDIN_FILENO) == 1) {
    if (!enabled_) {
      return;
    }
    tcgetattr(STDIN_FILENO, &original_);
    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    old_flags_ = flags;
  }

  ~RawTerminalGuard() {
    if (!enabled_) {
      return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_);
  }

 private:
  bool enabled_{false};
  int old_flags_{0};
  termios original_{};
};

void ClampUnit(double* value) {
  *value = std::clamp(*value, -1.0, 1.0);
}

void ApplyDecay(double step, double dt, double* value) {
  const double magnitude = std::max(0.0, std::abs(*value) - step * dt);
  *value = (*value >= 0.0) ? magnitude : -magnitude;
}

int64_t NowMicros() {
  static const auto start = std::chrono::steady_clock::now();
  const auto now = std::chrono::steady_clock::now() - start;
  return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const MovingTargetSimParams params =
      drake::yaml::LoadYamlFile<MovingTargetSimParams>(FLAGS_config);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  RawTerminalGuard terminal_guard;

  std::cout << "moving_target teleop config: " << FLAGS_config << "\n";
  std::cout << "Publishing teleop on " << params.lcm_channels.teleop_command << "\n";
  std::cout << "Keys: W/S or Up/Down = throttle, A/D or Left/Right = turn, Space = stop, Q = quit\n";

  double throttle = 0.0;
  double turn = 0.0;
  bool running = true;
  const double publish_period = 1.0 / std::max(1.0, params.teleop.publish_rate);

  while (running) {
    bool stop = false;
    char c = 0;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == '\033') {
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0 &&
            seq[0] == '[') {
          switch (seq[1]) {
            case 'A': throttle += params.teleop.throttle_step; break;
            case 'B': throttle -= params.teleop.throttle_step; break;
            case 'C': turn -= params.teleop.turn_step; break;
            case 'D': turn += params.teleop.turn_step; break;
            default: break;
          }
        }
        continue;
      }

      switch (c) {
        case 'w':
        case 'W': throttle += params.teleop.throttle_step; break;
        case 's':
        case 'S': throttle -= params.teleop.throttle_step; break;
        case 'a':
        case 'A': turn += params.teleop.turn_step; break;
        case 'd':
        case 'D': turn -= params.teleop.turn_step; break;
        case ' ': throttle = 0.0; turn = 0.0; stop = true; break;
        case 'q':
        case 'Q': running = false; break;
        default: break;
      }
    }

    ClampUnit(&throttle);
    ClampUnit(&turn);

    lcmt_moving_target_teleop_command msg{};
    msg.utime = NowMicros();
    msg.timestamp_millis = msg.utime / 1000;
    msg.throttle = throttle;
    msg.turn = turn;
    msg.stop = stop;
    const int encoded_size = msg.getEncodedSize();
    std::vector<uint8_t> bytes(static_cast<std::size_t>(encoded_size));
    msg.encode(bytes.data(), 0, encoded_size);
    lcm.Publish(params.lcm_channels.teleop_command, bytes.data(), encoded_size,
                std::nullopt);

    std::cout << "\rteleop throttle=" << throttle << " turn=" << turn
              << " stop=" << (stop ? "true " : "false") << std::flush;

    ApplyDecay(params.teleop.decay_rate, publish_period, &throttle);
    ApplyDecay(params.teleop.decay_rate, publish_period, &turn);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(publish_period));
  }

  std::cout << "\nExiting moving_target teleop.\n";
  return 0;
}

}  // namespace
}  // namespace uav_delivery

int main(int argc, char* argv[]) {
  return uav_delivery::DoMain(argc, argv);
}
