#include "decision_maker/parameter_estimator.hpp"
#include <cmath>
#include <algorithm>

namespace decision_maker
{

ParameterEstimator::ParameterEstimator()
{
  // Initialize with some default profiles
  TaskProfile default_profile;
  default_profile.avg_complexity_gflops = 5.0;
  default_profile.avg_local_time_sec = 0.5;
  default_profile.avg_edge_time_sec = 0.1;
  default_profile.sample_count = 0;

  profiles_["object_detection"] = default_profile;
  profiles_["path_planning"] = default_profile;
  profiles_["pointcloud_processing"] = default_profile;
}

task_offloading_core::ExecutionEstimate ParameterEstimator::estimate(
  const task_offloading_core::TaskInfo& task,
  const task_offloading_core::SystemState& system_state,
  const task_offloading_core::NetworkState& network_state)
{
  task_offloading_core::ExecutionEstimate estimate;

  // Get complexity (use provided or from profile)
  double complexity = task.complexity_gflops;
  if (complexity <= 0.0) {
    TaskProfile& profile = getProfile(task.task_type);
    complexity = profile.avg_complexity_gflops;
  }

  // Estimate times
  estimate.time_local_sec = estimateLocalTime(complexity, system_state);
  estimate.time_edge_sec = estimateEdgeTime(
    complexity, task.input_data_size, task.output_data_size, network_state);

  // Estimate energy consumption
  estimate.energy_local_j = estimateLocalEnergy(estimate.time_local_sec);
  
  double transfer_in_time = (task.input_data_size * 8.0) / 
                           (network_state.bandwidth_up_mbps * 1e6) +
                           network_state.latency_ms / 1000.0;
  double edge_compute_time = complexity / EDGE_PROCESSOR_GFLOPS;
  double transfer_out_time = (task.output_data_size * 8.0) /
                            (network_state.bandwidth_down_mbps * 1e6) +
                            network_state.latency_ms / 1000.0;
  
  estimate.energy_edge_j = estimateEdgeEnergy(
    transfer_in_time, edge_compute_time, transfer_out_time);

  // Estimate failure probabilities
  estimate.failure_prob_local = estimateFailureProbability(
    false, network_state, task.input_data_size);
  estimate.failure_prob_edge = estimateFailureProbability(
    true, network_state, task.input_data_size + task.output_data_size);

  // Confidence based on profile sample count
  TaskProfile& profile = getProfile(task.task_type);
  estimate.confidence = std::min(1.0, profile.sample_count / 20.0);

  return estimate;
}

void ParameterEstimator::updateProfile(
  const std::string& task_type,
  double complexity_gflops,
  double actual_time_sec,
  bool was_remote)
{
  TaskProfile& profile = getProfile(task_type);

  // Update complexity estimate (exponential moving average)
  if (profile.sample_count == 0) {
    profile.avg_complexity_gflops = complexity_gflops;
  } else {
    double alpha = 0.3; // Smoothing factor
    profile.avg_complexity_gflops = alpha * complexity_gflops +
                                   (1.0 - alpha) * profile.avg_complexity_gflops;
  }

  // Update time estimates
  if (was_remote) {
    if (profile.sample_count == 0) {
      profile.avg_edge_time_sec = actual_time_sec;
    } else {
      double alpha = 0.3;
      profile.avg_edge_time_sec = alpha * actual_time_sec +
                                 (1.0 - alpha) * profile.avg_edge_time_sec;
    }
  } else {
    if (profile.sample_count == 0) {
      profile.avg_local_time_sec = actual_time_sec;
    } else {
      double alpha = 0.3;
      profile.avg_local_time_sec = alpha * actual_time_sec +
                                  (1.0 - alpha) * profile.avg_local_time_sec;
    }
  }

  profile.sample_count++;
}

double ParameterEstimator::estimateLocalTime(
  double complexity_gflops,
  const task_offloading_core::SystemState& system_state) const
{
  // Base computation time
  double base_time = complexity_gflops / LOCAL_PROCESSOR_GFLOPS;

  // Adjust for CPU load (available CPU fraction)
  double available_cpu = 1.0 - system_state.cpu_usage;
  if (available_cpu < 0.1) available_cpu = 0.1; // Minimum 10%
  
  double adjusted_time = base_time / available_cpu;

  // Adjust for temperature throttling
  double temp_factor = getTemperatureFactor(system_state.cpu_temperature);
  adjusted_time *= temp_factor;

  return adjusted_time;
}

double ParameterEstimator::estimateEdgeTime(
  double complexity_gflops,
  uint64_t input_size,
  uint64_t output_size,
  const task_offloading_core::NetworkState& network_state) const
{
  // Transfer time for input
  double transfer_in_sec = (input_size * 8.0) / (network_state.bandwidth_up_mbps * 1e6);
  transfer_in_sec += network_state.latency_ms / 1000.0; // Add RTT/2

  // Computation time on edge (with GPU acceleration for suitable tasks)
  double compute_sec = complexity_gflops / (EDGE_PROCESSOR_GFLOPS * EDGE_GPU_ACCELERATION);

  // Assume small queue time for now (could be improved with actual queue info)
  double queue_sec = 0.05; // 50ms average queue time

  // Transfer time for output
  double transfer_out_sec = (output_size * 8.0) / (network_state.bandwidth_down_mbps * 1e6);
  transfer_out_sec += network_state.latency_ms / 1000.0;

  return transfer_in_sec + queue_sec + compute_sec + transfer_out_sec;
}

double ParameterEstimator::estimateLocalEnergy(double execution_time_sec) const
{
  // Energy = Power * Time
  // Assume CPU at ~70% load during computation
  double avg_power = POWER_IDLE_W + 0.7 * (POWER_COMPUTE_MAX_W - POWER_IDLE_W);
  return avg_power * execution_time_sec;
}

double ParameterEstimator::estimateEdgeEnergy(
  double transfer_in_time_sec,
  double compute_time_sec,
  double transfer_out_time_sec) const
{
  double energy = 0.0;

  // Energy for transmitting input
  energy += POWER_WIFI_TX_W * transfer_in_time_sec;

  // Energy for idle waiting during computation
  energy += POWER_IDLE_W * compute_time_sec;

  // Energy for receiving output
  energy += POWER_WIFI_RX_W * transfer_out_time_sec;

  return energy;
}

double ParameterEstimator::estimateFailureProbability(
  bool is_remote,
  const task_offloading_core::NetworkState& network_state,
  uint64_t data_size) const
{
  if (!is_remote) {
    // Local execution rarely fails (only hardware failures)
    return 1e-6;
  }

  // For remote execution, failure probability depends on network quality
  double base_failure_prob = 1e-4;

  // Increase probability based on packet loss
  double packet_loss_factor = 1.0 + network_state.packet_loss_rate * 100.0;

  // Calculate number of packets (assuming 1500 byte MTU)
  uint32_t num_packets = (data_size / 1500) + 1;

  // Probability of at least one packet loss
  double prob_no_loss = std::pow(1.0 - network_state.packet_loss_rate, num_packets);
  double prob_loss = 1.0 - prob_no_loss;

  // Combined failure probability
  double failure_prob = base_failure_prob * packet_loss_factor + prob_loss * 0.1;

  return std::min(0.5, failure_prob); // Cap at 50%
}

TaskProfile& ParameterEstimator::getProfile(const std::string& task_type)
{
  auto it = profiles_.find(task_type);
  if (it != profiles_.end()) {
    return it->second;
  }

  // Create default profile if not exists
  TaskProfile default_profile;
  default_profile.avg_complexity_gflops = 5.0;
  default_profile.avg_local_time_sec = 0.5;
  default_profile.avg_edge_time_sec = 0.1;
  default_profile.sample_count = 0;

  profiles_[task_type] = default_profile;
  return profiles_[task_type];
}

double ParameterEstimator::getTemperatureFactor(float temperature_celsius) const
{
  const float T_NOMINAL = 70.0f;
  const float T_MAX = 85.0f;

  if (temperature_celsius < T_NOMINAL) {
    return 1.0; // No throttling
  } else if (temperature_celsius < T_MAX) {
    // Linear degradation
    float factor = (T_MAX - temperature_celsius) / (T_MAX - T_NOMINAL);
    return 0.5 + 0.5 * factor; // Between 0.5 and 1.0
  } else {
    return 2.0; // Aggressive throttling (2x slower)
  }
}

} // namespace decision_maker