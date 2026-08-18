#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dairlib {
namespace simulation_gui {

// One numeric array in a middleware message. The GUI treats field_path as a
// hierarchy (for example, "joint_states/position") and does not need to know
// the concrete ROS 2 or LCM message type.
struct RobotStateSignalGroup {
  std::string source;
  std::string message_type;
  std::string field_path;
  std::vector<std::string> names;
  std::vector<double> values;
};

struct RobotStateSnapshot {
  int64_t timestamp_us{0};
  std::vector<RobotStateSignalGroup> signal_groups;
};

// One discoverable middleware source. The stable id is the wire-level topic
// or channel name; display_name and message_type are intended for the GUI.
struct RobotStateSourceInfo {
  std::string id;
  std::string display_name;
  std::string message_type;
  bool supported{false};
  bool selected{false};
};

struct RobotStateAdapterConfig {
  std::string transport{"ros2"};
  std::string lcm_url{"udpm://239.255.76.67:7667?ttl=2"};
  // Empty lists/names enable type-safe source discovery.
  std::vector<std::string> lcm_channels;
  std::string ros2_topic;
  double sample_rate_hz{120.0};
  size_t max_pending_samples{2400};
};

// Transport boundary for the GUI. Incoming states may arrive at 1 kHz, but
// TakeBatch returns a bounded, producer-side downsampled batch for each frame.
class RobotStateAdapter {
 public:
  virtual ~RobotStateAdapter() = default;

  virtual bool Start(std::string* error) = 0;
  virtual bool Poll(std::string* error) = 0;
  virtual std::vector<RobotStateSnapshot> TakeBatch() = 0;
  virtual std::vector<RobotStateSourceInfo> sources() const = 0;
  virtual bool SetSourceSelected(const std::string& source_id, bool selected,
                                 std::string* error) = 0;
  virtual std::string description() const = 0;
};

std::unique_ptr<RobotStateAdapter> MakeRobotStateAdapter(
    RobotStateAdapterConfig config, std::string* error);

}  // namespace simulation_gui
}  // namespace dairlib
