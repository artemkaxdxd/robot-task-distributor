cmake_minimum_required(VERSION 3.8)
project(task_offloading_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(task_offloading_interfaces REQUIRED)

# Create library for core components
add_library(task_offloading_core_lib
  src/models.cpp
  src/decision_model.cpp
)

ament_target_dependencies(task_offloading_core_lib
  rclcpp
  task_offloading_interfaces
)

target_include_directories(task_offloading_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install library
install(TARGETS task_offloading_core_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp task_offloading_interfaces)

ament_package()