cmake_minimum_required(VERSION 3.5)
project(communication_trace)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(communication_trace_msgs REQUIRED)

add_library(communication_trace src/comm_trace.cpp)
ament_target_dependencies(communication_trace rcl rclcpp std_msgs communication_trace_msgs)

include_directories(
  include/
  )

ament_export_targets(export_communication_trace HAS_LIBRARY_TARGET)
ament_export_dependencies(communication_trace_msgs rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
  )

install(
  TARGETS communication_trace
  EXPORT export_communication_trace
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
