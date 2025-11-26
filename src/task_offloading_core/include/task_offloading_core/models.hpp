#ifndef TASK_OFFLOADING_CORE__MODELS_HPP_
#define TASK_OFFLOADING_CORE__MODELS_HPP_

#include <string>

namespace task_offloading_core
{

// System state snapshot
struct SystemState
{
  float battery_level;      // 0.0 - 1.0
  float cpu_usage;          // 0.0 - 1.0
  float memory_usage;       // 0.0 - 1.0
  float cpu_temperature;    // Celsius
  uint32_t memory_available_mb;
};

// Network state snapshot
struct NetworkState
{
  float signal_strength_dbm;
  float bandwidth_up_mbps;
  float bandwidth_down_mbps;
  float latency_ms;
  float packet_loss_rate;   // 0.0 - 1.0
  bool edge_available;
};

// Task description
struct TaskInfo
{
  uint64_t task_id;
  std::string task_type;
  uint64_t input_data_size;
  uint64_t output_data_size;
  double complexity_gflops;
  double max_latency_sec;
  uint8_t priority;
  bool is_critical;
};

// Execution parameters estimation
struct ExecutionEstimate
{
  double time_local_sec;
  double time_edge_sec;
  double energy_local_j;
  double energy_edge_j;
  double failure_prob_local;
  double failure_prob_edge;
  double confidence;
};

// Decision result
struct OffloadingDecision
{
  bool execute_remotely;
  double estimated_time_sec;
  double estimated_energy_j;
  double confidence;
  std::string reasoning;
};

// Weight coefficients for objective function
struct WeightCoefficients
{
  double w_time;
  double w_energy;
  double w_risk;

  WeightCoefficients() : w_time(0.4), w_energy(0.4), w_risk(0.2) {}
  
  void normalize() {
    double sum = w_time + w_energy + w_risk;
    if (sum > 0.0) {
      w_time /= sum;
      w_energy /= sum;
      w_risk /= sum;
    }
  }
};

} // namespace task_offloading_core

#endif // TASK_OFFLOADING_CORE__MODELS_HPP_