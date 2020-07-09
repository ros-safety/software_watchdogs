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

# from launch import LaunchDescription
# from launch_ros.actions import LifecycleNode
# from launch_ros.actions import Node
##or:
import launch
import launch_ros
import lifecycle_msgs.msg

# def generate_launch_description():
#     return LaunchDescription([
#         LifecycleNode(package='lifecycle', node_executable='lifecycle_talker',
#                       node_name='lc_talker', output='screen'),
#         Node(package='lifecycle', node_executable='lifecycle_listener', output='screen'),
#         Node(package='lifecycle', node_executable='lifecycle_service_client', output='screen')
#     ])

def generate_launch_description():
    set_tty_launch_config_action = launch.actions.SetLaunchConfiguration("emulate_tty", "True")
    watchdog_node = launch_ros.actions.LifecycleNode(
        package='sw_watchdog',
        node_executable='windowed_watchdog',
        node_namespace='',
        node_name='windowed_watchdog',
        output='screen',
        arguments=['220', '3', '--publish', '--activate']
        #arguments=['__log_level:=debug']
    )
    return launch.LaunchDescription([set_tty_launch_config_action, watchdog_node])

#     configure_trans_event = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
#         lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/wiimote'),
#         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
#     ))
#     return launch.LaunchDescription([set_tty_launch_config_action, watchdog_node, configure_trans_event])
