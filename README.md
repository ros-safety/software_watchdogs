# SW Watchdog

A library of (software) watchdogs based on DDS Quality of Service (QoS) policies and ROS 2 [lifecycle nodes](https://github.com/ros2/demos/blob/master/lifecycle/README.rst).

This package includes a heartbeat node that can be added easily to an existing process via ROS 2 [node composition](https://index.ros.org/doc/ros2/Tutorials/Composition/).

## Overview

This package includes two watchdog implementations (*"readers"*) and a heartbeat node (a *"writer"*). A watchdog expects a heartbeat signal at the specified frequency and otherwise declares the *writer* to have failed. A failure results in the watchdog's life cycle [state machine](https://design.ros2.org/articles/node_lifecycle.html) to transition to the `Inactive` state along with emitting the corresponding state transition event. A system-level response can be implemented in the event handler to realize patterns such as cold standby, process restarts, *etc.*

* `SimpleWatchdog` declares the monitored topic as failed after a violation of the granted *lease* time. The implementation relies on the QoS liveliness policy provided by the `rmw` implementation.
* `WindowedWatchdog` declares the monitored topic as failed after the specified maximum number of *lease* violations. The implementation relies on the QoS liveliness and deadline policies provided by the `rmw` implementation.
* `SimpleHeartbeat` publishes a heartbeat signal at the specified frequency. The heartbeat signal asserts liveliness manually to the listening watchdog.

## Usage

The launch files included in this package demonstrate both node composition with a heartbeat signal and the configuration of a corresponding watchdog.

If you wish to use an `rmw`implementation other than the default, set the `RMW_IMPLEMENTATION` environment variable appropriately in all shells that you are using ROS in.

Then start the heartbeat and watchdog examples in separate terminals:
```
ros2 launch sw_watchdog heartbeat_composition.launch.py
ros2 launch sw_watchdog watchdog_lifecycle.launch.py

```
The first command composes a single process consisting of a ROS 2 `demo_nodes_cpp::Talker` with a `SimpleHeartbeat` set at *200ms*. The second command starts a `SimpleWatchdog` which grants a lease of *220ms* to the Heartbeat publisher. The watchdog will transition to the `Inactive` state as soon as the Heartbeat publisher violates the lease (*e.g.,* via CTRL+C in the first terminal). Since the watchdog is a lifecycle node, it can be re-activated to listen for a Heartbeat signal via:
```
ros2 lifecycle set watchdog activate
```

To test the `WindowedWatchdog` replace the launch command in the second terminal with:
```
ros2 launch sw_watchdog windowed_watchdog_lifecycle.launch.py
```
This grants the Heartbeat publisher a maximum of three deadline misses. Deadline misses can be tested by inserting artificial delays in the publishing thread, for example.

It is important that compatible *lease times* are configured for the Heartbeat signal and the watchdog. DDS does not establish a connection when incompatible QoS times are chosen (Cyclone and Connext DDS additionally display a warning message when this is the case):
* The liveliness lease time specified for the watchdog should be `>=` than that specified for the Heartbeat publisher.

## Requirements

This package includes custom messages.
If you are compiling it from source and wish to use a non-default `rmw` implementation, you must have the appropriate `rmw` packages installed when you compile this package.
See [Install DDS implementations](https://index.ros.org/doc/ros2/Installation/DDS-Implementations/) for more information on installing alternative `rmw` implementations.

To use the `heartbeat_composition.launch.py` example, the `ros-*-demo-nodes-cpp` must be installed.

## Compatibility

This code is built and tested under:

* [ROS 2 Dashing Diademata](https://index.ros.org/doc/ros2/Installation/Dashing/) with [Ubuntu 18.04.4](http://releases.ubuntu.com/18.04/)  
  _**Note:** Ouptut may be delayed / buffered in Dashing when using `ros2 launch` (see https://answers.ros.org/question/332829/no-stdout-logging-output-in-ros2-using-launch/)_
* [ROS 2 Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/) with [Ubuntu 20.04](http://releases.ubuntu.com/20.04/)

The following DDS `rmw` [implementations](https://index.ros.org/doc/ros2/Concepts/DDS-and-ROS-middleware-implementations/) were tested in both environments (via the default Ubuntu packages that ship with the Dashing and Foxy releases):
* [Fast DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds)  
  _**Note:** In Dashing you must use Fast DDS, rather than the default Fast RTPS (which is a prior version of Fast DDS). Fast RTPS does not implement QoS._
* [Connext DDS](https://www.rti.com/products/) v5.3.1
* [Cyclone DDS](https://projects.eclipse.org/projects/iot.cyclonedds)
  _**Note:** In order to be compatible with the `WindowedWatchdog`, the liveliness lease duration (not deadline) configured for the Heartbeat publisher has to account for the number of permitted violations on the watchdog (reader) side! E.g., if the watchdog is configured for a lease of *220ms* with *3* violations, the heartbeat should be set to a liveliness lease duration of (at least) *3 x 220ms* (requires a manual patch). Otherwise, the watchdog will declare the monitored node as failed immediately after the first lease violation._

## TODO

The `Heartbeat` message defined in this package supports the notion of *checkpoints*. Future watchdog versions could add support for control flow monitoring based on this information.

## Contact

For any questions or comments, please post a question at [ROS Answers](http://answers.ros.org/questions) following the [ROS support guidelines](http://wiki.ros.org/Support).
[Add the `safety_wg` tag](https://answers.ros.org/questions/ask/?tags=safety_wg) to your question and someone from the Safety working group will spot it more easily.
