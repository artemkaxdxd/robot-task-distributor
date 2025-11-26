#ifndef TASK_OFFLOADING_CORE__DECISION_MODEL_HPP_
#define TASK_OFFLOADING_CORE__DECISION_MODEL_HPP_

#include "task_offloading_core/models.hpp"
#include <memory>

namespace task_offloading_core
{

class DecisionModel
{
public:
  DecisionModel();
  ~DecisionModel() = default;

  // Make offloading decision for a task
  OffloadingDecision decide(
    const TaskInfo& task,
    const SystemState& system_state,
    const NetworkState& network_state,
    const ExecutionEstimate& estimate);

  // Adapt weight coefficients based on current state
  WeightCoefficients adaptWeights(
    const SystemState& system_state,
    const NetworkState& network_state,
    bool is_critical_task) const;

  // Calculate objective function value
  double calculateObjective(
    double time_norm,
    double energy_norm,
    double risk,
    const WeightCoefficients& weights) const;

private:
  // Normalization parameters (learned from data)
  double time_min_sec_;
  double time_max_sec_;
  double energy_min_j_;
  double energy_max_j_;

  // Threshold values
  static constexpr float BATTERY_CRITICAL = 0.2f;
  static constexpr float SIGNAL_WEAK_DBM = -70.0f;
  static constexpr float PACKET_LOSS_HIGH = 0.05f;

  // Normalize value to [0, 1] range
  double normalize(double value, double min_val, double max_val) const;

  // Sigmoid function for smooth transitions
  double sigmoid(double x) const;
};

} // namespace task_offloading_core

#endif // TASK_OFFLOADING_CORE__DECISION_MODEL_HPP_