#ifndef TASK_EXECUTOR__COMPUTATION_MODULES_HPP_
#define TASK_EXECUTOR__COMPUTATION_MODULES_HPP_

#include "task_offloading_interfaces/msg/task_parameters.hpp"
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <map>

namespace task_executor
{

// Base class for computation modules
class ComputationModule
{
public:
  virtual ~ComputationModule() = default;

  // Execute the task
  virtual std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) = 0;

  // Estimate computational complexity in GFLOPS
  virtual double estimateComplexity(
    const task_offloading_interfaces::msg::TaskParameters& params) const = 0;

  // Get module name
  virtual std::string getName() const = 0;
};

// Object detection module (simulated)
class ObjectDetectionModule : public ComputationModule
{
public:
  ObjectDetectionModule(bool use_gpu = false);

  std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) override;

  double estimateComplexity(
    const task_offloading_interfaces::msg::TaskParameters& params) const override;

  std::string getName() const override { return "object_detection"; }

private:
  bool use_gpu_;
  
  // Simulate detection processing
  void simulateDetection(int width, int height, float threshold);
};

// Path planning module (simulated)
class PathPlanningModule : public ComputationModule
{
public:
  PathPlanningModule();

  std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) override;

  double estimateComplexity(
    const task_offloading_interfaces::msg::TaskParameters& params) const override;

  std::string getName() const override { return "path_planning"; }

private:
  // Simulate path planning
  void simulatePlanning(int map_size);
};

// Point cloud processing module (simulated)
class PointCloudModule : public ComputationModule
{
public:
  PointCloudModule();

  std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) override;

  double estimateComplexity(
    const task_offloading_interfaces::msg::TaskParameters& params) const override;

  std::string getName() const override { return "pointcloud_processing"; }

private:
  // Simulate point cloud processing
  void simulateProcessing(int num_points, float voxel_size);
};

// Module registry
class ModuleRegistry
{
public:
  static ModuleRegistry& getInstance();

  void registerModule(const std::string& name, std::shared_ptr<ComputationModule> module);
  std::shared_ptr<ComputationModule> getModule(const std::string& name);
  bool hasModule(const std::string& name) const;

private:
  ModuleRegistry() = default;
  std::map<std::string, std::shared_ptr<ComputationModule>> modules_;
};

} // namespace task_executor

#endif // TASK_EXECUTOR__COMPUTATION_MODULES_HPP_