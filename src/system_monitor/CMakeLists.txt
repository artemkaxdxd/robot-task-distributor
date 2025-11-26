cmake_minimum_required(VERSION 3.8)
project(system_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(task_offloading_interfaces REQUIRED)

# System monitor node
add_executable(system_monitor_node
  src/system_monitor_node.cpp
  src/system_monitor.cpp
)

ament_target_dependencies(system_monitor_node
  rclcpp
  task_offloading_interfaces
)

target_include_directories(system_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add macOS-specific frameworks
if(APPLE)
  target_link_libraries(system_monitor_node
    "-framework IOKit"
    "-framework CoreFoundation"
  )
endif()

# Install targets
install(TARGETS
  system_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()