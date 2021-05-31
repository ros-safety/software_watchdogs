# Copyright (c) 2021 Robert Bosch GmbH
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

import time

from lifecycle_msgs.srv import GetState
import rclpy
from ros2component.api import get_components_in_container, unload_component_from_container


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('shutdown_heartbeat')
    cli = node.create_client(GetState, 'simple_watchdog/get_state')

    while not cli.wait_for_service(timeout_sec=1.0):
        print('simple watchdog not yet available, waiting...')

    active = False
    req = GetState.Request()
    while not active:
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            time.sleep(1)
            node.get_logger().info(
                'Watchdog active, so now shutting down talker and heartbeat to test watchdog.')
            active = True

            container = 'my_namespace/my_container'
            comps = get_components_in_container(node=node, remote_container_node_name=container)
            print(comps)
            for uid, error, reason in unload_component_from_container(
                                          node=node,
                                          remote_container_node_name=container,
                                          component_uids=[1, 2]):
                if error:
                    return 'Failed to unload component {} from `{}` container node\n    {}'.format(
                        uid, container, reason.capitalize()
                    )
                print('Unloaded component {} from `{}` container node'.format(
                    uid, container
                ))
        else:
            node.get_logger().error('Exception while calling service: %r' % future.exception())
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
