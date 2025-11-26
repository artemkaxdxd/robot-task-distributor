cmake_minimum_required(VERSION 3.8)
project(network_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(task_offloading_interfaces REQUIRED)

# Network monitor node
add_executable(network_monitor_node
  src/network_monitor_node.cpp
  src/network_monitor.cpp
)

ament_target_dependencies(network_monitor_node
  rclcpp
  task_offloading_interfaces
)

target_include_directories(network_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  network_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()