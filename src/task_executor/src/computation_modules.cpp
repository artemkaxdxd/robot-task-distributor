#include "task_executor/computation_modules.hpp"
#include <thread>
#include <random>
#include <cmath>

namespace task_executor
{

// Object Detection Module Implementation

ObjectDetectionModule::ObjectDetectionModule(bool use_gpu)
: use_gpu_(use_gpu)
{
}

std::vector<uint8_t> ObjectDetectionModule::execute(
  const task_offloading_interfaces::msg::TaskParameters& params,
  const std::vector<uint8_t>& input_data)
{
  // Simulate object detection processing
  simulateDetection(params.image_width, params.image_height, params.confidence_threshold);

  // Create dummy output (in real implementation, would return detection results)
  std::string result = "Detected objects: ";
  for (const auto& cls : params.target_classes) {
    result += cls + " ";
  }

  return std::vector<uint8_t>(result.begin(), result.end());
}

double ObjectDetectionModule::estimateComplexity(
  const task_offloading_interfaces::msg::TaskParameters& params) const
{
  // Complexity proportional to image size
  // Typical object detector: ~5-15 GFLOPS for 640x480 image
  double pixels = params.image_width * params.image_height;
  double base_gflops = (pixels / (640.0 * 480.0)) * 10.0;
  
  return base_gflops;
}

void ObjectDetectionModule::simulateDetection(int width, int height, float threshold)
{
  // Simulate processing time based on image size
  double pixels = width * height;
  double base_time_ms = (pixels / 1e6) * 100.0; // 100ms per megapixel base
  
  if (use_gpu_) {
    base_time_ms /= 10.0; // GPU is ~10x faster
  }

  // Add some randomness
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.8, 1.2);
  
  double actual_time_ms = base_time_ms * dis(gen);
  
  // Sleep to simulate computation
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(actual_time_ms)));
}

// Path Planning Module Implementation

PathPlanningModule::PathPlanningModule()
{
}

std::vector<uint8_t> PathPlanningModule::execute(
  const task_offloading_interfaces::msg::TaskParameters& params,
  const std::vector<uint8_t>& input_data)
{
  // Simulate path planning
  int map_size = 1000; // Default map size
  simulatePlanning(map_size);

  // Create dummy output
  std::string result = "Planned path with " + std::to_string(map_size) + " nodes";
  return std::vector<uint8_t>(result.begin(), result.end());
}

double PathPlanningModule::estimateComplexity(
  const task_offloading_interfaces::msg::TaskParameters& params) const
{
  // Complexity depends on search space size
  // Typical A* or RRT: 2-10 GFLOPS depending on map size
  return 5.0;
}

void PathPlanningModule::simulatePlanning(int map_size)
{
  // Simulate planning time
  double base_time_ms = (map_size / 1000.0) * 50.0; // 50ms for 1000 nodes
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.8, 1.2);
  
  double actual_time_ms = base_time_ms * dis(gen);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(actual_time_ms)));
}

// Point Cloud Module Implementation

PointCloudModule::PointCloudModule()
{
}

std::vector<uint8_t> PointCloudModule::execute(
  const task_offloading_interfaces::msg::TaskParameters& params,
  const std::vector<uint8_t>& input_data)
{
  // Simulate point cloud processing
  simulateProcessing(params.num_points, params.voxel_size);

  // Create dummy output
  std::string result = "Processed " + std::to_string(params.num_points) + " points";
  return std::vector<uint8_t>(result.begin(), result.end());
}

double PointCloudModule::estimateComplexity(
  const task_offloading_interfaces::msg::TaskParameters& params) const
{
  // Complexity proportional to number of points
  // Typical processing: ~5-20 GFLOPS for 100k points
  double gflops = (params.num_points / 100000.0) * 10.0;
  return gflops;
}

void PointCloudModule::simulateProcessing(int num_points, float voxel_size)
{
  // Simulate processing time
  double base_time_ms = (num_points / 100000.0) * 80.0; // 80ms for 100k points
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.8, 1.2);
  
  double actual_time_ms = base_time_ms * dis(gen);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(actual_time_ms)));
}

// Module Registry Implementation

ModuleRegistry& ModuleRegistry::getInstance()
{
  static ModuleRegistry instance;
  return instance;
}

void ModuleRegistry::registerModule(const std::string& name, 
                                     std::shared_ptr<ComputationModule> module)
{
  modules_[name] = module;
}

std::shared_ptr<ComputationModule> ModuleRegistry::getModule(const std::string& name)
{
  auto it = modules_.find(name);
  if (it != modules_.end()) {
    return it->second;
  }
  return nullptr;
}

bool ModuleRegistry::hasModule(const std::string& name) const
{
  return modules_.find(name) != modules_.end();
}

} // namespace task_executor