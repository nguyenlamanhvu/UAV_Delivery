#pragma once

#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace uav_delivery {
namespace systems {

template <typename Message>
class Subscriber final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Subscriber);

  Subscriber(drake::lcm::DrakeLcmInterface* lcm, const std::string& channel) {
    subscription_ = drake::lcm::Subscribe<Message>(
        lcm, channel,
        [data = data_](const Message& message) {
          data->message = message;
          ++data->count;
        });
    if (subscription_ != nullptr) {
      subscription_->set_unsubscribe_on_delete(true);
    }
  }

  const Message& message() const { return data_->message; }
  int64_t count() const { return data_->count; }

  void clear() {
    data_->message = {};
    data_->count = 0;
  }

 private:
  struct Data {
    Message message{};
    int64_t count{0};
  };

  std::shared_ptr<Data> data_{std::make_shared<Data>()};
  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface> subscription_;
};

template <typename Predicate>
void LcmHandleSubscriptionsUntil(drake::lcm::DrakeLcm* lcm,
                                 Predicate done,
                                 std::function<bool()> should_stop = {}) {
  while (!done()) {
    if (should_stop && should_stop()) {
      return;
    }
    lcm->HandleSubscriptions(10);
  }
}

template <typename InputMessageType>
class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop);

  LcmDrivenLoop(drake::lcm::DrakeLcm* lcm,
                std::shared_ptr<drake::systems::Diagram<double>> diagram,
                const drake::systems::LeafSystem<double>* lcm_parser,
                const std::string& input_channel,
                bool is_forced_publish)
      : lcm_(lcm),
        diagram_(std::move(diagram)),
        diagram_ptr_(diagram_.get()),
        lcm_parser_(lcm_parser),
        input_channel_(input_channel),
        input_sub_(lcm, input_channel),
        is_forced_publish_(is_forced_publish),
        simulator_(*diagram_ptr_) {}

  drake::systems::Diagram<double>* get_diagram() { return diagram_ptr_; }

  drake::systems::Context<double>& get_diagram_mutable_context() {
    return simulator_.get_mutable_context();
  }

  void Simulate(double end_time = std::numeric_limits<double>::infinity(),
                std::function<bool()> should_stop = {}) {
    auto& diagram_context = simulator_.get_mutable_context();

    std::cout << "Waiting for first LCM message on " << input_channel_
              << std::endl;
    LcmHandleSubscriptionsUntil(lcm_, [&]() { return input_sub_.count() > 0; },
                                should_stop);
    if (should_stop && should_stop()) {
      return;
    }

    double time = input_sub_.message().utime * 1e-6;
    diagram_context.SetTime(time);
    simulator_.Initialize();

    while (time < end_time) {
      LcmHandleSubscriptionsUntil(lcm_, [&]() { return input_sub_.count() > 0; },
                                  should_stop);
      if (should_stop && should_stop()) {
        return;
      }

      while (lcm_->HandleSubscriptions(0) > 0) {
      }

      const InputMessageType message = input_sub_.message();
      input_sub_.clear();

      if (lcm_parser_ != nullptr) {
        lcm_parser_->get_input_port(0).FixValue(
            &diagram_ptr_->GetMutableSubsystemContext(*lcm_parser_,
                                                      &diagram_context),
            message);
      }

      time = message.utime * 1e-6;
      if (time > simulator_.get_context().get_time() + 1.0 ||
          time < simulator_.get_context().get_time()) {
        std::cout << "Driven loop time reset from "
                  << simulator_.get_context().get_time() << " to " << time
                  << std::endl;
        simulator_.get_mutable_context().SetTime(time);
        simulator_.Initialize();
      }

      simulator_.AdvanceTo(time);
      diagram_ptr_->CalcForcedUnrestrictedUpdate(
          diagram_context, &diagram_context.get_mutable_state());
      diagram_ptr_->CalcForcedDiscreteVariableUpdate(
          diagram_context, &diagram_context.get_mutable_discrete_state());
      if (is_forced_publish_) {
        diagram_ptr_->ForcedPublish(diagram_context);
      }
    }
  }

 private:
  drake::lcm::DrakeLcm* lcm_{};
  std::shared_ptr<drake::systems::Diagram<double>> diagram_;
  drake::systems::Diagram<double>* diagram_ptr_{};
  const drake::systems::LeafSystem<double>* lcm_parser_{};
  std::string input_channel_;
  Subscriber<InputMessageType> input_sub_;
  bool is_forced_publish_{};
  drake::systems::Simulator<double> simulator_;
};

}  // namespace systems
}  // namespace uav_delivery
