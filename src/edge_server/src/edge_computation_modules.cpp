#include "edge_server/edge_computation_modules.hpp"
#include <thread>
#include <random>
#include <cmath>

namespace edge_server
{

// Edge Object Detection Module

EdgeObjectDetectionModule::EdgeObjectDetectionModule()
{
}

std::vector<uint8_t> EdgeObjectDetectionModule::execute(
  const task_offloading_interfaces::msg::TaskParameters& params,
  const std::vector<uint8_t>& input_data)
{
  // Simulate GPU-accelerated object detection
  simulateGPUDetection(params.image_width, params.image_height, params.confidence_threshold);

  // Create dummy output
  std::string result = "GPU Detected objects: ";
  for (const auto& cls : params.target_classes) {
    result += cls + " ";
  }

  return std::vector<uint8_t>(result.begin(), result.end());
}

void EdgeObjectDetectionModule::simulateGPUDetection(int width, int height, float threshold)
{
  // GPU is ~10x faster than CPU
  double pixels = width * height;
  double base_time_ms = (pixels / 1e6) * 10.0; // 10ms per megapixel with GPU

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.8, 1.2);
  
  double actual_time_ms = base_time_ms * dis(gen);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(actual_time_ms)));
}

// Edge Path Planning Module

EdgePathPlanningModule::EdgePathPlanningModule()
{
}

std::vector<uint8_t> EdgePathPlanningModule::execute(
  const task_offloading_interfaces::msg::TaskParameters& params,
  const std::vector<uint8_t>& input_data)
{
  // Simulate faster path planning on edge server
  int map_size = 1000;
  double base_time_ms = (map_size / 1000.0) * 30.0; // Faster than local

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.8, 1.2);
  
  double actual_time_ms = base_time_ms * dis(gen);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(actual_time_ms)));

  std::string result = "Edge planned path with " + std::to_string(map_size) + " nodes";
  return std::vector<uint8_t>(result.begin(), result.end());
}

// Edge Point Cloud Module

EdgePointCloudModule::EdgePointCloudModule()
{
}

std::vector<uint8_t> EdgePointCloudModule::execute(
  const task_offloading_interfaces::msg::TaskParameters& params,
  const std::vector<uint8_t>& input_data)
{
  // Simulate GPU-accelerated point cloud processing
  simulateGPUProcessing(params.num_points, params.voxel_size);

  std::string result = "GPU processed " + std::to_string(params.num_points) + " points";
  return std::vector<uint8_t>(result.begin(), result.end());
}

void EdgePointCloudModule::simulateGPUProcessing(int num_points, float voxel_size)
{
  // GPU accelerated processing
  double base_time_ms = (num_points / 100000.0) * 15.0; // Faster than CPU

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.8, 1.2);
  
  double actual_time_ms = base_time_ms * dis(gen);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(actual_time_ms)));
}

// Edge Module Registry

EdgeModuleRegistry& EdgeModuleRegistry::getInstance()
{
  static EdgeModuleRegistry instance;
  return instance;
}

void EdgeModuleRegistry::registerModule(const std::string& name,
                                        std::shared_ptr<EdgeComputationModule> module)
{
  modules_[name] = module;
}

std::shared_ptr<EdgeComputationModule> EdgeModuleRegistry::getModule(const std::string& name)
{
  auto it = modules_.find(name);
  if (it != modules_.end()) {
    return it->second;
  }
  return nullptr;
}

bool EdgeModuleRegistry::hasModule(const std::string& name) const
{
  return modules_.find(name) != modules_.end();
}

} // namespace edge_server