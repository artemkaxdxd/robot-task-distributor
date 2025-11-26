cmake_minimum_required(VERSION 3.8)
project(task_offloading_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemState.msg"
  "msg/NetworkState.msg"
  "msg/TaskParameters.msg"
  "msg/ExecutionMetrics.msg"
  "msg/EdgeStationStatus.msg"
  "srv/DecideOffloading.srv"
  "srv/ExecuteTask.srv"
  "srv/RecordExecution.srv"
  "srv/EstimateParameters.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
