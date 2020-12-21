cmake_minimum_required(VERSION 3.5)
project(sw_watchdog_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heartbeat.msg"
  "msg/Status.msg"
  DEPENDENCIES std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
