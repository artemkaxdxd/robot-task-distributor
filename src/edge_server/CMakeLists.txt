cmake_minimum_required(VERSION 3.8)
project(edge_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(task_offloading_interfaces REQUIRED)
find_package(task_offloading_core REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Edge server node
add_executable(edge_server_node
  src/edge_server_node.cpp
  src/task_scheduler.cpp
  src/edge_computation_modules.cpp
)

ament_target_dependencies(edge_server_node
  rclcpp
  task_offloading_interfaces
  task_offloading_core
  cv_bridge
  sensor_msgs
)

target_link_libraries(edge_server_node
  ${OpenCV_LIBS}
)

target_include_directories(edge_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Station monitor node
add_executable(station_monitor_node
  src/station_monitor_node.cpp
)

ament_target_dependencies(station_monitor_node
  rclcpp
  task_offloading_interfaces
)

# Install targets
install(TARGETS
  edge_server_node
  station_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()