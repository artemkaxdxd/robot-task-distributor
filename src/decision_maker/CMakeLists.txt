cmake_minimum_required(VERSION 3.8)
project(decision_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(task_offloading_interfaces REQUIRED)
find_package(task_offloading_core REQUIRED)

# Decision maker node
add_executable(decision_maker_node
  src/decision_maker_node.cpp
  src/parameter_estimator.cpp
)

ament_target_dependencies(decision_maker_node
  rclcpp
  task_offloading_interfaces
  task_offloading_core
)

target_include_directories(decision_maker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  decision_maker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()