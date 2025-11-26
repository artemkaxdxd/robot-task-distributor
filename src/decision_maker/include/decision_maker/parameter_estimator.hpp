#ifndef DECISION_MAKER__PARAMETER_ESTIMATOR_HPP_
#define DECISION_MAKER__PARAMETER_ESTIMATOR_HPP_

#include "task_offloading_core/models.hpp"
#include <map>
#include <string>

namespace decision_maker
{

// Simple profiling data for a task type
struct TaskProfile
{
  double avg_complexity_gflops;
  double avg_local_time_sec;
  double avg_edge_time_sec;
  int sample_count;
};

class ParameterEstimator
{
public:
  ParameterEstimator();
  ~ParameterEstimator() = default;

  // Estimate execution parameters for a task
  task_offloading_core::ExecutionEstimate estimate(
    const task_offloading_core::TaskInfo& task,
    const task_offloading_core::SystemState& system_state,
    const task_offloading_core::NetworkState& network_state);

  // Update profile based on actual execution
  void updateProfile(
    const std::string& task_type,
    double complexity_gflops,
    double actual_time_sec,
    bool was_remote);

private:
  // Task profiles database (in-memory for now)
  std::map<std::string, TaskProfile> profiles_;

  // Hardware characteristics
  static constexpr double LOCAL_PROCESSOR_GFLOPS = 10.0;   // Raspberry Pi 4 ~10 GFLOPS
  static constexpr double EDGE_PROCESSOR_GFLOPS = 200.0;   // Server with GPU ~200+ GFLOPS
  static constexpr double EDGE_GPU_ACCELERATION = 10.0;    // GPU speedup factor

  // Power consumption models (Watts)
  static constexpr double POWER_IDLE_W = 2.5;
  static constexpr double POWER_COMPUTE_MAX_W = 7.5;
  static constexpr double POWER_WIFI_TX_W = 1.8;
  static constexpr double POWER_WIFI_RX_W = 1.0;

  // Estimate local execution time
  double estimateLocalTime(
    double complexity_gflops,
    const task_offloading_core::SystemState& system_state) const;

  // Estimate edge execution time (including transfer)
  double estimateEdgeTime(
    double complexity_gflops,
    uint64_t input_size,
    uint64_t output_size,
    const task_offloading_core::NetworkState& network_state) const;

  // Estimate local energy consumption
  double estimateLocalEnergy(double execution_time_sec) const;

  // Estimate edge energy consumption
  double estimateEdgeEnergy(
    double transfer_in_time_sec,
    double compute_time_sec,
    double transfer_out_time_sec) const;

  // Estimate failure probability
  double estimateFailureProbability(
    bool is_remote,
    const task_offloading_core::NetworkState& network_state,
    uint64_t data_size) const;

  // Get or create profile for task type
  TaskProfile& getProfile(const std::string& task_type);

  // Temperature throttling factor
  double getTemperatureFactor(float temperature_celsius) const;
};

} // namespace decision_maker

#endif // DECISION_MAKER__PARAMETER_ESTIMATOR_HPP_