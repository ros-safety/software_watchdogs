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

import launch
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg

def generate_launch_description():
    set_tty_launch_config_action = launch.actions.SetLaunchConfiguration("emulate_tty", "True")

    # Launch Description
    ld = launch.LaunchDescription()

    # Start monitored entity once (via docker)
    docker_run_cmd = launch.actions.ExecuteProcess(
        cmd=['docker', 'run', '-d', '--name', 'talker', '-v', '/var/run/docker.sock:/var/run/docker.sock', '-v', '/usr/bin/docker:/usr/bin/docker', 'sw_watchdogs:latest', 'ros2', 'launch', 'sw_watchdog', 'heartbeat_composition.launch.py'],
        #output='screen'
    )

    # Restart monitored entity upon watchdog miss (via docker)
    docker_restart_cmd = launch.actions.ExecuteProcess(
        cmd=['docker', 'restart', 'talker']
        #output='screen'
    )

    # Remove docker container
    docker_rm_cmd = launch.actions.ExecuteProcess(
        cmd=['docker', 'rm', '-f', 'talker']
        #output='screen'
    )

    # Shutdown event
    #shutdown_event = EmitEvent( event = launch.events.Shutdown() )

    # Watchdog node
    watchdog_node = LifecycleNode(
        package='sw_watchdog',
        node_executable='simple_watchdog',
        node_namespace='',
        node_name='simple_watchdog',
        output='screen',
        arguments=['220', '--publish', '--activate']
        #arguments=['__log_level:=debug']
    )

    # Make the Watchdog node take the 'activate' transition
    watchdog_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(watchdog_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    # When the watchdog reaches the 'inactive' state, log a message
    # and restart monitored entity (via docker)
    watchdog_inactive_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = watchdog_node,
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "Watchdog transitioned to 'INACTIVE' state." ),
                # Change state event (inactive -> active)
                watchdog_activate_trans_event,
                # Restart the monitored entity
                docker_restart_cmd,
            ],
        )
    )

    # When Shutdown is requested, clean up docker
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown = [
                # Log
                LogInfo( msg = "Launch was asked to shutdown." ),
                # Clean up docker
                docker_rm_cmd,
            ],
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed
    ld.add_action( set_tty_launch_config_action )
    ld.add_action( docker_run_cmd )
    ld.add_action( watchdog_node )
    ld.add_action( watchdog_inactive_handler )
    ld.add_action( shutdown_handler )

    return ld
