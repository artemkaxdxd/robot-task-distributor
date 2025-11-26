cmake_minimum_required(VERSION 3.8)
project(task_executor)

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

# Task executor node
add_executable(task_executor_node
  src/task_executor_node.cpp
  src/computation_modules.cpp
)

ament_target_dependencies(task_executor_node
  rclcpp
  task_offloading_interfaces
  task_offloading_core
  cv_bridge
  sensor_msgs
)

target_link_libraries(task_executor_node
  ${OpenCV_LIBS}
)

target_include_directories(task_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Task generator test node
add_executable(task_generator_node
  test/task_generator_node.cpp
)

ament_target_dependencies(task_generator_node
  rclcpp
  task_offloading_interfaces
)

install(TARGETS
  task_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()