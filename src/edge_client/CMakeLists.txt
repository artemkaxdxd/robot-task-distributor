cmake_minimum_required(VERSION 3.8)
project(edge_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(task_offloading_interfaces REQUIRED)

# Edge connection tester
add_executable(edge_connection_tester
  src/edge_connection_tester.cpp
)

target_include_directories(edge_connection_tester PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(edge_connection_tester
  rclcpp
  task_offloading_interfaces
)

# Install targets
install(TARGETS
  edge_connection_tester
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()