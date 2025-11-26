#ifndef EDGE_SERVER__EDGE_COMPUTATION_MODULES_HPP_
#define EDGE_SERVER__EDGE_COMPUTATION_MODULES_HPP_

#include "task_offloading_interfaces/msg/task_parameters.hpp"
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace edge_server
{

// Base class for edge computation modules (with GPU support)
class EdgeComputationModule
{
public:
  virtual ~EdgeComputationModule() = default;

  // Execute the task
  virtual std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) = 0;

  // Get module name
  virtual std::string getName() const = 0;

  // Does module support GPU acceleration
  virtual bool supportsGPU() const { return false; }
};

// Edge object detection module (GPU accelerated)
class EdgeObjectDetectionModule : public EdgeComputationModule
{
public:
  EdgeObjectDetectionModule();

  std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) override;

  std::string getName() const override { return "object_detection"; }
  bool supportsGPU() const override { return true; }

private:
  void simulateGPUDetection(int width, int height, float threshold);
};

// Edge path planning module
class EdgePathPlanningModule : public EdgeComputationModule
{
public:
  EdgePathPlanningModule();

  std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) override;

  std::string getName() const override { return "path_planning"; }
};

// Edge point cloud processing module
class EdgePointCloudModule : public EdgeComputationModule
{
public:
  EdgePointCloudModule();

  std::vector<uint8_t> execute(
    const task_offloading_interfaces::msg::TaskParameters& params,
    const std::vector<uint8_t>& input_data) override;

  std::string getName() const override { return "pointcloud_processing"; }
  bool supportsGPU() const override { return true; }

private:
  void simulateGPUProcessing(int num_points, float voxel_size);
};

// Edge module registry
class EdgeModuleRegistry
{
public:
  static EdgeModuleRegistry& getInstance();

  void registerModule(const std::string& name, 
                     std::shared_ptr<EdgeComputationModule> module);
  std::shared_ptr<EdgeComputationModule> getModule(const std::string& name);
  bool hasModule(const std::string& name) const;

private:
  EdgeModuleRegistry() = default;
  std::map<std::string, std::shared_ptr<EdgeComputationModule>> modules_;
};

} // namespace edge_server

#endif // EDGE_SERVER__EDGE_COMPUTATION_MODULES_HPP_