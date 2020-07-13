# SW Watchdog

A set of (software) watchdogs based on DDS Quality of Service (QoS) policies and ROS 2 [lifecycle nodes](https://github.com/ros2/demos/blob/master/lifecycle/README.rst).

Package includes a heartbeat node that can be simply added to an existing process via ROS 2 [node composition](https://index.ros.org/doc/ros2/Tutorials/Composition/).

## Overview

This package includes two watchdog implementations (think of as *"readers"*) and a heartbeat node (a *"writer"*). Given a specific configuration, a watchdog expects a heartbeat signal at the specified frequency and otherwise declares the *writer* to have failed. A failure results in the watchdog's life cycle [state machine](https://design.ros2.org/articles/node_lifecycle.html) to transition to the `Inactive` state along with emitting the corresponding state transition event. A system-level response can be implemented in the event handler to improve system robustness via patterns such as cold standby, process restarts, etc.

* `SimpleWatchdog` declares the monitored topic as failed after a violation of the granted *lease* duration. The implementation relies on the QoS liveliness policy provided by the `rmw` implementation.
* `WindowedWatchdog` declares the monitored topic as failed after the specified number of *lease* violations. The implementation relies on the QoS liveliness and deadline policies provided by the `rmw` implementation.
* `SimpleHeartbeat` publishes a heartbeat signal at the specified frequency. The heartbeat signal asserts (QoS) liveliness manually to the watchdog. 

## Usage

Easiest 

## Requirements

This package has the following dependency in addition to standard ROS 2 dependencies:


## Compatibility

This code is built and tested under:

* [ROS 2 Dashing Diademata](https://index.ros.org/doc/ros2/Installation/Dashing/) with [Ubuntu 18.04.4](http://releases.ubuntu.com/18.04/)
* [ROS 2 Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/) with [Ubuntu 20.04](http://releases.ubuntu.com/20.04/)

The following DDS implementations 

## TODO

## Contact

For any questions or comments, please reach out to <philipp@mapless.ai>.
