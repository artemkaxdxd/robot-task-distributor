#include "task_offloading_core/decision_model.hpp"
#include <cmath>
#include <algorithm>

namespace task_offloading_core
{

DecisionModel::DecisionModel()
: time_min_sec_(0.01),    // 10 ms minimum
  time_max_sec_(5.0),     // 5 seconds maximum
  energy_min_j_(0.1),     // 0.1 J minimum
  energy_max_j_(50.0)     // 50 J maximum
{
}

OffloadingDecision DecisionModel::decide(
  const TaskInfo& task,
  const SystemState& system_state,
  const NetworkState& network_state,
  const ExecutionEstimate& estimate)
{
  OffloadingDecision decision;

  // Step 1: Check if task is critical and has strict latency requirement
  if (task.is_critical && task.max_latency_sec < 0.1) {
    // Critical tasks with very strict latency must execute locally
    decision.execute_remotely = false;
    decision.estimated_time_sec = estimate.time_local_sec;
    decision.estimated_energy_j = estimate.energy_local_j;
    decision.confidence = 1.0;
    decision.reasoning = "Critical task with strict latency requirement";
    return decision;
  }

  // Step 2: Check if edge is available
  if (!network_state.edge_available) {
    decision.execute_remotely = false;
    decision.estimated_time_sec = estimate.time_local_sec;
    decision.estimated_energy_j = estimate.energy_local_j;
    decision.confidence = 1.0;
    decision.reasoning = "Edge station not available";
    return decision;
  }

  // Step 3: Check if edge execution violates latency constraint
  if (estimate.time_edge_sec > task.max_latency_sec) {
    if (estimate.time_local_sec <= task.max_latency_sec) {
      decision.execute_remotely = false;
      decision.estimated_time_sec = estimate.time_local_sec;
      decision.estimated_energy_j = estimate.energy_local_j;
      decision.confidence = 0.9;
      decision.reasoning = "Edge execution exceeds latency constraint";
      return decision;
    } else {
      // Both violate constraint, choose edge (faster)
      decision.execute_remotely = true;
      decision.estimated_time_sec = estimate.time_edge_sec;
      decision.estimated_energy_j = estimate.energy_edge_j;
      decision.confidence = 0.5;
      decision.reasoning = "Both options exceed latency, choosing faster (edge)";
      return decision;
    }
  }

  // Step 4: Multi-criteria optimization
  WeightCoefficients weights = adaptWeights(system_state, network_state, task.is_critical);

  // Normalize parameters
  double time_local_norm = normalize(estimate.time_local_sec, time_min_sec_, time_max_sec_);
  double time_edge_norm = normalize(estimate.time_edge_sec, time_min_sec_, time_max_sec_);
  double energy_local_norm = normalize(estimate.energy_local_j, energy_min_j_, energy_max_j_);
  double energy_edge_norm = normalize(estimate.energy_edge_j, energy_min_j_, energy_max_j_);

  // Calculate objective functions
  double obj_local = calculateObjective(
    time_local_norm, energy_local_norm, 
    estimate.failure_prob_local, weights);
  
  double obj_edge = calculateObjective(
    time_edge_norm, energy_edge_norm,
    estimate.failure_prob_edge, weights);

  // Choose option with lower objective value
  if (obj_local < obj_edge) {
    decision.execute_remotely = false;
    decision.estimated_time_sec = estimate.time_local_sec;
    decision.estimated_energy_j = estimate.energy_local_j;
    decision.reasoning = "Local execution optimal (obj=" + 
                        std::to_string(obj_local) + " < " + std::to_string(obj_edge) + ")";
  } else {
    decision.execute_remotely = true;
    decision.estimated_time_sec = estimate.time_edge_sec;
    decision.estimated_energy_j = estimate.energy_edge_j;
    decision.reasoning = "Edge execution optimal (obj=" + 
                        std::to_string(obj_edge) + " < " + std::to_string(obj_local) + ")";
  }

  // Calculate confidence based on difference in objectives
  double obj_diff = std::abs(obj_local - obj_edge);
  decision.confidence = std::min(1.0, obj_diff * 2.0); // Scale difference to confidence

  return decision;
}

WeightCoefficients DecisionModel::adaptWeights(
  const SystemState& system_state,
  const NetworkState& network_state,
  bool is_critical_task) const
{
  WeightCoefficients weights;

  // Start with base weights
  weights.w_time = 0.4;
  weights.w_energy = 0.4;
  weights.w_risk = 0.2;

  // Adapt for low battery
  if (system_state.battery_level < BATTERY_CRITICAL) {
    double battery_factor = sigmoid((BATTERY_CRITICAL - system_state.battery_level) / 0.05);
    weights.w_energy = 0.4 + 0.2 * battery_factor; // Increase to 0.6
    weights.w_time = 0.4 - 0.1 * battery_factor;   // Decrease to 0.3
    weights.w_risk = 0.2 - 0.1 * battery_factor;   // Decrease to 0.1
  }

  // Adapt for poor network quality
  bool poor_network = (network_state.signal_strength_dbm < SIGNAL_WEAK_DBM) ||
                      (network_state.packet_loss_rate > PACKET_LOSS_HIGH);
  
  if (poor_network) {
    double network_factor = 0.5;
    if (network_state.signal_strength_dbm < SIGNAL_WEAK_DBM) {
      network_factor = std::max(network_factor,
        sigmoid((SIGNAL_WEAK_DBM - network_state.signal_strength_dbm) / 10.0));
    }
    if (network_state.packet_loss_rate > PACKET_LOSS_HIGH) {
      network_factor = std::max(network_factor,
        sigmoid((network_state.packet_loss_rate - PACKET_LOSS_HIGH) / 0.05));
    }

    weights.w_risk = 0.2 + 0.3 * network_factor;   // Increase to 0.5
    weights.w_time = 0.4 - 0.1 * network_factor;   // Decrease to 0.3
    weights.w_energy = 0.4 - 0.2 * network_factor; // Decrease to 0.2
  }

  // Adapt for critical tasks
  if (is_critical_task) {
    weights.w_time = 0.7;
    weights.w_energy = 0.1;
    weights.w_risk = 0.2;
  }

  weights.normalize();
  return weights;
}

double DecisionModel::calculateObjective(
  double time_norm,
  double energy_norm,
  double risk,
  const WeightCoefficients& weights) const
{
  return weights.w_time * time_norm +
         weights.w_energy * energy_norm +
         weights.w_risk * risk;
}

double DecisionModel::normalize(double value, double min_val, double max_val) const
{
  if (max_val <= min_val) {
    return 0.5;
  }
  double normalized = (value - min_val) / (max_val - min_val);
  return std::max(0.0, std::min(1.0, normalized));
}

double DecisionModel::sigmoid(double x) const
{
  return 1.0 / (1.0 + std::exp(-x));
}

} // namespace task_offloading_core