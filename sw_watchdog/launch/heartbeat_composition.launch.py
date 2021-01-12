# Copyright (c) 2020 Mapless AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a heartbeat in a component container."""

import subprocess
import os

import launch
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.event_handlers.on_shutdown import OnShutdown

# Hack to cleanly exit all roslaunch group processes (docker init is GID 1)
def group_stop(context, *args, **kwargs):
    gid = os.getpgid(os.getpid())
    subprocess.call(['kill', '-INT', '--', f"-{gid}"])

# Note: syntax has changed in foxy (removal of 'node_' prefixes)
def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='my_container',
            node_namespace='my_namespace',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='demo_nodes_cpp',
                    node_plugin='demo_nodes_cpp::Talker',
                    node_name='talker'),
                ComposableNode(
                    package='sw_watchdog',
                    node_plugin='sw_watchdog::SimpleHeartbeat',
                    node_name='heartbeat',
                    parameters=[{'period': 200}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='screen'
    )

    # When Shutdown is requested (launch), clean up all child processes
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown = [
                # Log
                LogInfo( msg = "heartbeat_composition was asked to shutdown." ),
                # Clean up
                OpaqueFunction(function=group_stop),
            ],
        )
    )

    return launch.LaunchDescription([container, shutdown_handler])
