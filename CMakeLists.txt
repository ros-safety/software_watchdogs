cmake_minimum_required(VERSION 3.5)
project(sw_watchdog)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(sw_watchdog_msgs
  "msg/Heartbeat.msg"
  "msg/Status.msg"
  DEPENDENCIES std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

### nodes
add_library(${PROJECT_NAME} SHARED
  src/simple_heartbeat.cpp
  src/simple_watchdog.cpp
  src/windowed_watchdog.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "rcutils"
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "SW_WATCHDOG_BUILDING_DLL")
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::SimpleHeartbeat"
  EXECUTABLE simple_heartbeat)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::SimpleWatchdog"
  EXECUTABLE simple_watchdog)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::WindowedWatchdog"
  EXECUTABLE windowed_watchdog)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ros_testing REQUIRED)
#   add_ros_test(
#     test/test_watchdog.py
#     TIMEOUT 60
#   )
# endif()

# To enable use of heartbeat and status messages in library
rosidl_target_interfaces(sw_watchdog sw_watchdog_msgs "rosidl_typesupport_cpp")

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
