// Copyright (c) 2020 Mapless AI, Inc.
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
#include "rcutils/cmdline_parser.h"
#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcutils/logging_macros.h"

#include "sw_watchdog/msg/heartbeat.hpp"
#include "sw_watchdog/visibility_control.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

constexpr char OPTION_AUTO_START[] = "--autostart";
constexpr char DEFAULT_TOPIC_NAME[] = "heartbeat";

namespace {

void print_usage()
{
    std::cout <<
        "Usage: simple_watchdog lease [" << OPTION_AUTO_START << "] [-h]\n\n"
        "required arguments:\n"
        "\tlease: Lease in positive integer milliseconds granted to the watched entity.\n"
        "optional arguments:\n"
        "\t" << OPTION_AUTO_START << ": Start the watchdog on creation.  Defaults to false.\n"
        "\t-h : Print this help message." <<
        std::endl;
}

} // anonymous ns

namespace sw_watchdog
{


/// SimpleWatchdog inheriting from rclcpp_lifecycle::LifecycleNode
class SimpleWatchdog : public rclcpp_lifecycle::LifecycleNode
{
public:
    SW_WATCHDOG_PUBLIC
    explicit SimpleWatchdog(const rclcpp::NodeOptions& options)
        : rclcpp_lifecycle::LifecycleNode("simple_watchdog", options),
          autostart_(false), topic_name_(DEFAULT_TOPIC_NAME), qos_profile_(10)
    {
        // Parse node arguments
        const std::vector<std::string>& args = this->get_node_options().arguments();
        std::vector<char *> cargs;
        cargs.reserve(args.size());
        for(size_t i = 0; i < args.size(); ++i)
            cargs.push_back(const_cast<char*>(args[i].c_str()));

        if(args.size() < 1 || rcutils_cli_option_exist(&cargs[0], &cargs[0] + cargs.size(), "-h")) {
            print_usage();
            // TODO: Update the rclcpp_components template to be able to handle
            // exceptions. Raise one here, so stack unwinding happens gracefully.
            std::exit(0);
        }

        // Lease duration must be >= heartbeat's lease duration
        lease_duration_ = std::chrono::milliseconds(std::stoul(args[1]));

        if(rcutils_cli_option_exist(&cargs[0], &cargs[0] + cargs.size(), OPTION_AUTO_START))
            autostart_ = true;
    }

    /// Transition callback for state configuring
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &)
    {
        // Initialize and configure node
        qos_profile_
            .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
            .liveliness_lease_duration(lease_duration_);

        heartbeat_sub_options_.event_callbacks.liveliness_callback =
            [](rclcpp::QOSLivelinessChangedInfo &event) {
                printf("Reader Liveliness changed event: \n");
                printf("  alive_count: %d\n", event.alive_count);
                printf("  not_alive_count: %d\n", event.not_alive_count);
                printf("  alive_count_change: %d\n", event.alive_count_change);
                printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
            };

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state activating
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &)
    {
        if (!heartbeat_sub_) {
            heartbeat_sub_ = create_subscription<sw_watchdog::msg::Heartbeat>(
                topic_name_,
                qos_profile_,
                [this](const typename sw_watchdog::msg::Heartbeat::SharedPtr msg) -> void {
                    RCLCPP_INFO(get_logger(), "Watchdog raised, heartbeat sent at [%f]", msg->stamp);
                },
                heartbeat_sub_options_);
        }

        // Starting from this point, all messages are sent to the network.
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state deactivating
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        // Starting from this point, listening to the heartbeat signal is disabled
        //pub_->on_deactivate(); // XXX there does not seem to be an equivalent for subscribers.
        heartbeat_sub_.reset();
        heartbeat_sub_ = nullptr;

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state cleaningup
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called (no-op).");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state shutting down
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &state)
    {
        heartbeat_sub_.reset();
        heartbeat_sub_ = nullptr;

        RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    /// The lease duration granted to the remote (heartbeat) publisher
    std::chrono::milliseconds lease_duration_;
    rclcpp::Subscription<sw_watchdog::msg::Heartbeat>::SharedPtr heartbeat_sub_ = nullptr;
    bool autostart_;
    const std::string topic_name_;
    rclcpp::QoS qos_profile_;
    rclcpp::SubscriptionOptions heartbeat_sub_options_;
};

} // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(sw_watchdog::SimpleWatchdog)
