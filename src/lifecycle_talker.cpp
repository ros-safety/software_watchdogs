// Copyright (c) 2020 Mapless AI, Inc. Based on code that is
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "sw_watchdog/msg/dummy_message.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/// LifecycleTalker inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * The lifecycle talker does not like the regular "talker" node
 * inherit from node, but rather from lifecyclenode. This brings
 * in a set of callbacks which are getting invoked depending on
 * the current state of the node.
 */
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// LifecycleTalker constructor
    explicit LifecycleTalker(const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}

    /// Callback for walltimer in order to publish the message.
    void publish()
    {
        static size_t count = 0;
        auto msg = std::make_unique<sw_watchdog::msg::DummyMessage>();
        msg->header = std_msgs::msg::Header{};
        msg->header.stamp = this->now();
        msg->data = "Hello, world! " + std::to_string(count++);

        // Print the current state for demo purposes
        if (!pub_->is_activated()) {
            RCLCPP_INFO(get_logger(),
                        "Lifecycle publisher is currently inactive. Messages are not published.");
        } else {
            RCLCPP_INFO(get_logger(),
                        "Lifecycle publisher is active. Publishing: [%s]",
                        msg->data.c_str());
        }

        // Only if the publisher is in an active state, the message transfer is
        // enabled and the message actually published.
        pub_->publish(std::move(msg));
  }

  /// Transition callback for state configuring
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &)
  {
      // Initialize and configure node
      pub_ = this->create_publisher<sw_watchdog::msg::DummyMessage>("topic", 10); /* QoS history_depth */
      timer_ = this->create_wall_timer(500ms, std::bind(&LifecycleTalker::publish, this));

      RCLCPP_INFO(get_logger(), "on_configure() is called.");

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state activating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &)
  {
      // Starting from this point, all messages are sent to the network.
      pub_->on_activate();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

      // Sleep for 2 seconds to emulate some important work in the activating phase.
      std::this_thread::sleep_for(2s);

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state deactivating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &)
  {
      // Starting from this point, all messages are no longer sent to the network.
      pub_->on_deactivate();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state cleaningup
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &)
  {
      // Release the shared pointers to the timer and publisher
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state shutting down
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &state)
  {
      // Release the shared pointers to the timer and publisher
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
    // This lifecycle publisher can be activated or deactivated regarding on which state the lifecycle node
    // is in. By default, a lifecycle publisher is inactive by creation and has to be activated to publish.
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sw_watchdog::msg::DummyMessage>> pub_;
    /// Timer that periodically triggers the publish function.
    std::shared_ptr<rclcpp::TimerBase> timer_;
};

// A lifecycle node has the same node API as a regular node.
int main(int argc, char * argv[])
{
    // force flush of the stdout buffer (ensures a correct sync of all prints even when executed simultaneously
    // within the launch file)
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    std::shared_ptr<LifecycleTalker> lc_node = std::make_shared<LifecycleTalker>("lc_talker");

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
