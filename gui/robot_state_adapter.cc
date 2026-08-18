#include "gui/robot_state_adapter.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <lcm/lcm-cpp.hpp>

#include "uav_delivery/lcmt_quadrotor_state.hpp"
#include "uav_delivery/lcmt_moving_target_state.hpp"


namespace dairlib {
namespace simulation_gui {
namespace {

void NormalizeNames(const std::string& prefix, size_t value_count,
                    std::vector<std::string>* names) {
  names->resize(value_count);
  for (size_t i = 0; i < value_count; ++i) {
    if ((*names)[i].empty()) {
      (*names)[i] = prefix + "[" + std::to_string(i) + "]";
    }
  }
}

RobotStateSignalGroup MakeSignalGroup(
    std::string source, std::string message_type, std::string field_path,
    std::vector<std::string> names, std::vector<double> values) {
  const size_t slash = field_path.find_last_of('/');
  const std::string leaf_name =
      slash == std::string::npos ? field_path : field_path.substr(slash + 1);
  NormalizeNames(leaf_name, values.size(), &names);
  return RobotStateSignalGroup{
      std::move(source), std::move(message_type), std::move(field_path),
      std::move(names), std::move(values)};
}

void AppendSnapshot(const RobotStateSnapshot& source,
                    RobotStateSnapshot* destination) {
  destination->timestamp_us =
      std::max(destination->timestamp_us, source.timestamp_us);
  destination->signal_groups.insert(destination->signal_groups.end(),
                                    source.signal_groups.begin(),
                                    source.signal_groups.end());
}

std::string ToDdsTopic(const std::string& topic) {
  if (topic.rfind("rt/", 0) == 0) return topic;
  if (!topic.empty() && topic.front() == '/') return "rt" + topic;
  return "rt/" + topic;
}

std::string FromDdsTopic(const std::string& topic) {
  if (topic.rfind("rt/", 0) == 0) return "/" + topic.substr(3);
  return topic;
}

// Downsample on the producer side, then preserve samples across render stalls.
class BufferedRobotStateAdapter : public RobotStateAdapter {
 public:
  BufferedRobotStateAdapter(double sample_rate_hz, size_t max_pending_samples)
      : sample_period_us_(std::max<int64_t>(
            1, std::llround(1000000.0 / sample_rate_hz))),
        max_pending_samples_(std::max<size_t>(1, max_pending_samples)) {}

  std::vector<RobotStateSnapshot> TakeBatch() final {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::vector<RobotStateSnapshot> result;
    result.reserve(pending_samples_.size());
    while (!pending_samples_.empty()) {
      result.push_back(std::move(pending_samples_.front()));
      pending_samples_.pop_front();
    }
    return result;
  }

 protected:
  void StoreSample(RobotStateSnapshot snapshot) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const int64_t timestamp_us = snapshot.timestamp_us;
    if (!has_next_sample_time_ || timestamp_us < previous_input_time_us_) {
      pending_samples_.clear();
      next_sample_time_us_ = timestamp_us;
      has_next_sample_time_ = true;
    }
    previous_input_time_us_ = timestamp_us;
    if (timestamp_us < next_sample_time_us_) return;

    if (pending_samples_.size() == max_pending_samples_) {
      pending_samples_.pop_front();
    }
    pending_samples_.push_back(std::move(snapshot));

    const int64_t periods =
        (timestamp_us - next_sample_time_us_) / sample_period_us_ + 1;
    next_sample_time_us_ += periods * sample_period_us_;
  }

 private:
  const int64_t sample_period_us_;
  const size_t max_pending_samples_;
  std::mutex queue_mutex_;
  std::deque<RobotStateSnapshot> pending_samples_;
  int64_t previous_input_time_us_{0};
  int64_t next_sample_time_us_{0};
  bool has_next_sample_time_{false};
};

class LcmRobotStateAdapter final : public BufferedRobotStateAdapter {
 public:
  LcmRobotStateAdapter(std::string lcm_url, std::vector<std::string> channels,
                       double sample_rate_hz, size_t max_pending_samples)
      : BufferedRobotStateAdapter(sample_rate_hz, max_pending_samples),
        lcm_url_(std::move(lcm_url)),
        channels_(std::move(channels)),
        auto_discovery_(channels_.empty()) {}

  ~LcmRobotStateAdapter() final {
    running_.store(false);
    if (receiver_thread_.joinable()) receiver_thread_.join();
  }

  bool Start(std::string* error) final {
    lcm_ = std::make_unique<lcm::LCM>(lcm_url_);
    if (!lcm_->good()) {
      *error = "Failed to initialize LCM at " + lcm_url_;
      return false;
    }
    const std::vector<std::string> subscriptions =
        auto_discovery_ ? std::vector<std::string>{".*"} : channels_;
    if (!auto_discovery_) {
      std::lock_guard<std::mutex> lock(source_mutex_);
      for (const std::string& channel : channels_) {
        selected_sources_[channel] = true;
      }
    }
    for (const std::string& channel : subscriptions) {
      auto* subscription = lcm_->subscribe(
          channel, &LcmRobotStateAdapter::HandleRawMessage, this);
      const int queue_capacity = auto_discovery_ ? 256 : 8;
      if (subscription == nullptr ||
          subscription->setQueueCapacity(queue_capacity) != 0) {
        *error = "Failed to configure LCM subscription on " + channel;
        return false;
      }
    }
    running_.store(true);
    receiver_thread_ = std::thread([this] { ReceiveLoop(); });
    return true;
  }

  bool Poll(std::string* error) final {
    if (receive_failed_.load()) {
      *error = "LCM robot-state receive failed";
      return false;
    }
    return true;
  }

  std::string description() const final {
    if (auto_discovery_) {
      return "LCM auto-discovery (Quadrotor/Target)";
    }
    return "LCM " + std::to_string(channels_.size()) +
           " robot component channels";
  }

  std::vector<RobotStateSourceInfo> sources() const final {
    std::vector<RobotStateSourceInfo> result;
    std::lock_guard<std::mutex> lock(source_mutex_);
    result.reserve(selected_sources_.size());
    for (const auto& [channel, selected] : selected_sources_) {
      result.push_back(RobotStateSourceInfo{
          channel, channel, "lcm_state", true, selected});
    }
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b) {
                return a.display_name < b.display_name;
              });
    return result;
  }

  bool SetSourceSelected(const std::string& source_id, bool selected,
                         std::string* error) final {
    std::lock_guard<std::mutex> lock(source_mutex_);
    const auto source = selected_sources_.find(source_id);
    if (source == selected_sources_.end()) {
      *error = "Unknown LCM channel '" + source_id + "'";
      return false;
    }
    source->second = selected;
    if (!selected) latest_components_.erase(source_id);
    return true;
  }

 private:
  void ReceiveLoop() {
    while (running_.load()) {
      if (lcm_->handleTimeout(50) < 0) {
        receive_failed_.store(true);
        running_.store(false);
      }
    }
  }

  void HandleRawMessage(const lcm::ReceiveBuffer* buffer,
                        const std::string& channel) {
    if (buffer == nullptr) return;
    RobotStateSnapshot component;
    
    uav_delivery::lcmt_quadrotor_state quad_msg;
    if (quad_msg.decode(buffer->data, 0, static_cast<int>(buffer->data_size)) >= 0) {
      component.timestamp_us = quad_msg.utime;
      component.signal_groups.push_back(MakeSignalGroup(
          channel, "lcmt_quadrotor_state", "position", {"x", "y", "z"},
          {quad_msg.position[0], quad_msg.position[1], quad_msg.position[2]}));
      component.signal_groups.push_back(MakeSignalGroup(
          channel, "lcmt_quadrotor_state", "velocity", {"vx", "vy", "vz"},
          {quad_msg.velocity[0], quad_msg.velocity[1], quad_msg.velocity[2]}));
      component.signal_groups.push_back(MakeSignalGroup(
          channel, "lcmt_quadrotor_state", "rpy", {"roll", "pitch", "yaw"},
          {quad_msg.rpy[0], quad_msg.rpy[1], quad_msg.rpy[2]}));
      component.signal_groups.push_back(MakeSignalGroup(
          channel, "lcmt_quadrotor_state", "angular_velocity", {"wx", "wy", "wz"},
          {quad_msg.body_angular_velocity[0], quad_msg.body_angular_velocity[1], quad_msg.body_angular_velocity[2]}));
    } else {
      uav_delivery::lcmt_moving_target_state target_msg;
      if (target_msg.decode(buffer->data, 0, static_cast<int>(buffer->data_size)) >= 0) {
        component.timestamp_us = target_msg.utime;
        component.signal_groups.push_back(MakeSignalGroup(
            channel, "lcmt_moving_target_state", "position", {"x", "y", "yaw"},
            {target_msg.x, target_msg.y, target_msg.yaw}));
        component.signal_groups.push_back(MakeSignalGroup(
            channel, "lcmt_moving_target_state", "velocity", {"vx", "vy", "yaw_rate"},
            {target_msg.vx, target_msg.vy, target_msg.yaw_rate}));
        component.signal_groups.push_back(MakeSignalGroup(
            channel, "lcmt_moving_target_state", "wheels", {"lf", "lr", "rf", "rr"},
            {target_msg.left_wheel_angle_front, target_msg.left_wheel_angle_rear, target_msg.right_wheel_angle_front, target_msg.right_wheel_angle_rear}));
      } else {
        return;
      }
    }
    RobotStateSnapshot combined;
    {
      std::lock_guard<std::mutex> lock(source_mutex_);
      const auto [source, inserted] =
          selected_sources_.try_emplace(channel, false);
      if (inserted && auto_discovery_) channels_.push_back(channel);
      if (!source->second) return;
      latest_components_[channel] = std::move(component);

      for (const auto& [configured_channel, selected] : selected_sources_) {
        if (!selected) continue;
        const auto entry = latest_components_.find(configured_channel);
        if (entry != latest_components_.end()) {
          AppendSnapshot(entry->second, &combined);
        }
      }
    }
    if (!combined.signal_groups.empty()) StoreSample(std::move(combined));
  }

  std::string lcm_url_;
  std::vector<std::string> channels_;
  const bool auto_discovery_;
  mutable std::mutex source_mutex_;
  std::unordered_map<std::string, bool> selected_sources_;
  std::unordered_map<std::string, RobotStateSnapshot> latest_components_;
  std::unique_ptr<lcm::LCM> lcm_;
  std::atomic<bool> running_{false};
  std::atomic<bool> receive_failed_{false};
  std::thread receiver_thread_;
};


}  // namespace

std::unique_ptr<RobotStateAdapter> MakeRobotStateAdapter(
    RobotStateAdapterConfig config, std::string* error) {
  if (!std::isfinite(config.sample_rate_hz) || config.sample_rate_hz <= 0.0 ||
      config.max_pending_samples == 0) {
    *error = "Adapter sample rate and pending-sample capacity must be positive";
    return nullptr;
  }
  if (config.transport == "lcm") {
    return std::make_unique<LcmRobotStateAdapter>(
        std::move(config.lcm_url), std::move(config.lcm_channels),
        config.sample_rate_hz, config.max_pending_samples);
  }
  if (config.transport == "ros2" || config.transport == "fastdds") {
    *error = "ROS 2/FastDDS transport is not supported in the UAV Delivery project (LCM only).";
    return nullptr;
  }
  *error = "Unknown robot-state transport '" + config.transport +
           "' (expected ros2, fastdds, or lcm)";
  return nullptr;
}

}  // namespace simulation_gui
}  // namespace dairlib
