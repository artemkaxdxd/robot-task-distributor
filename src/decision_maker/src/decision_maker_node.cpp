#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/srv/decide_offloading.hpp"
#include "task_offloading_interfaces/srv/estimate_parameters.hpp"
#include "task_offloading_interfaces/msg/system_state.hpp"
#include "task_offloading_interfaces/msg/network_state.hpp"
#include "decision_maker/parameter_estimator.hpp"
#include "task_offloading_core/decision_model.hpp"
#include "task_offloading_core/models.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class DecisionMakerNode : public rclcpp::Node
{
public:
  DecisionMakerNode()
  : Node("decision_maker_node")
  {
    // Create estimator and decision model
    estimator_ = std::make_unique<decision_maker::ParameterEstimator>();
    decision_model_ = std::make_unique<task_offloading_core::DecisionModel>();

    // Subscribe to system and network state
    system_state_sub_ = this->create_subscription<task_offloading_interfaces::msg::SystemState>(
      "/robot/system_state", 10,
      std::bind(&DecisionMakerNode::systemStateCallback, this, std::placeholders::_1));

    network_state_sub_ = this->create_subscription<task_offloading_interfaces::msg::NetworkState>(
      "/robot/network_state", 10,
      std::bind(&DecisionMakerNode::networkStateCallback, this, std::placeholders::_1));

    // Create services
    decide_service_ = this->create_service<task_offloading_interfaces::srv::DecideOffloading>(
      "/offloading/decide",
      std::bind(&DecisionMakerNode::handleDecideRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    estimate_service_ = this->create_service<task_offloading_interfaces::srv::EstimateParameters>(
      "/offloading/estimate",
      std::bind(&DecisionMakerNode::handleEstimateRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Decision Maker Node started");
  }

private:
  void systemStateCallback(const task_offloading_interfaces::msg::SystemState::SharedPtr msg)
  {
    last_system_state_ = *msg;
    has_system_state_ = true;
  }

  void networkStateCallback(const task_offloading_interfaces::msg::NetworkState::SharedPtr msg)
  {
    last_network_state_ = *msg;
    has_network_state_ = true;
  }

  void handleDecideRequest(
    const std::shared_ptr<task_offloading_interfaces::srv::DecideOffloading::Request> request,
    std::shared_ptr<task_offloading_interfaces::srv::DecideOffloading::Response> response)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Check if we have state information
    if (!has_system_state_ || !has_network_state_) {
      RCLCPP_WARN(this->get_logger(), 
                  "Cannot make decision: missing system or network state");
      response->execute_remotely = false;
      response->confidence = 0.0;
      response->reasoning = "Missing system or network state information";
      return;
    }

    // Convert ROS messages to internal structures
    task_offloading_core::TaskInfo task;
    task.task_id = request->task_id;
    task.task_type = request->task_type;
    task.input_data_size = request->input_data_size;
    task.output_data_size = request->output_data_size;
    task.complexity_gflops = request->parameters.complexity_estimate;
    task.max_latency_sec = request->max_latency_sec;
    task.priority = request->priority;
    task.is_critical = request->is_critical;

    task_offloading_core::SystemState system_state;
    system_state.battery_level = last_system_state_.battery_level;
    system_state.cpu_usage = last_system_state_.cpu_usage;
    system_state.memory_usage = last_system_state_.memory_usage;
    system_state.cpu_temperature = last_system_state_.cpu_temperature;
    system_state.memory_available_mb = last_system_state_.memory_available_mb;

    task_offloading_core::NetworkState network_state;
    network_state.signal_strength_dbm = last_network_state_.signal_strength_dbm;
    network_state.bandwidth_up_mbps = last_network_state_.bandwidth_up_mbps;
    network_state.bandwidth_down_mbps = last_network_state_.bandwidth_down_mbps;
    network_state.latency_ms = last_network_state_.latency_ms;
    network_state.packet_loss_rate = last_network_state_.packet_loss_rate;
    network_state.edge_available = last_network_state_.edge_available;

    // Estimate parameters
    auto estimate = estimator_->estimate(task, system_state, network_state);

    // Make decision
    auto decision = decision_model_->decide(task, system_state, network_state, estimate);

    // Fill response
    response->execute_remotely = decision.execute_remotely;
    response->estimated_time_local_sec = estimate.time_local_sec;
    response->estimated_time_edge_sec = estimate.time_edge_sec;
    response->estimated_energy_local_j = estimate.energy_local_j;
    response->estimated_energy_edge_j = estimate.energy_edge_j;
    response->confidence = decision.confidence;
    response->reasoning = decision.reasoning;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_INFO(this->get_logger(),
                "Decision for task %lu (%s): %s (confidence: %.2f, time: %ld μs)",
                request->task_id,
                request->task_type.c_str(),
                decision.execute_remotely ? "EDGE" : "LOCAL",
                decision.confidence,
                duration.count());

    RCLCPP_DEBUG(this->get_logger(),
                 "  Estimates: Local(%.3fs, %.2fJ) Edge(%.3fs, %.2fJ)",
                 estimate.time_local_sec, estimate.energy_local_j,
                 estimate.time_edge_sec, estimate.energy_edge_j);
  }

  void handleEstimateRequest(
    const std::shared_ptr<task_offloading_interfaces::srv::EstimateParameters::Request> request,
    std::shared_ptr<task_offloading_interfaces::srv::EstimateParameters::Response> response)
  {
    // Convert ROS messages to internal structures
    task_offloading_core::TaskInfo task;
    task.task_type = request->task_type;
    task.complexity_gflops = request->parameters.complexity_estimate;
    task.input_data_size = 1024 * 1024; // Assume 1MB for estimation
    task.output_data_size = 100 * 1024; // Assume 100KB output

    task_offloading_core::SystemState system_state;
    system_state.battery_level = request->current_system_state.battery_level;
    system_state.cpu_usage = request->current_system_state.cpu_usage;
    system_state.memory_usage = request->current_system_state.memory_usage;
    system_state.cpu_temperature = request->current_system_state.cpu_temperature;
    system_state.memory_available_mb = request->current_system_state.memory_available_mb;

    task_offloading_core::NetworkState network_state;
    network_state.signal_strength_dbm = request->current_network_state.signal_strength_dbm;
    network_state.bandwidth_up_mbps = request->current_network_state.bandwidth_up_mbps;
    network_state.bandwidth_down_mbps = request->current_network_state.bandwidth_down_mbps;
    network_state.latency_ms = request->current_network_state.latency_ms;
    network_state.packet_loss_rate = request->current_network_state.packet_loss_rate;
    network_state.edge_available = request->current_network_state.edge_available;

    // Estimate parameters
    auto estimate = estimator_->estimate(task, system_state, network_state);

    // Fill response
    response->estimated_time_local_sec = estimate.time_local_sec;
    response->estimated_time_edge_sec = estimate.time_edge_sec;
    response->estimated_energy_local_j = estimate.energy_local_j;
    response->estimated_energy_edge_j = estimate.energy_edge_j;
    response->estimated_failure_prob_local = estimate.failure_prob_local;
    response->estimated_failure_prob_edge = estimate.failure_prob_edge;
    response->confidence = estimate.confidence;
  }

  // Subscribers
  rclcpp::Subscription<task_offloading_interfaces::msg::SystemState>::SharedPtr system_state_sub_;
  rclcpp::Subscription<task_offloading_interfaces::msg::NetworkState>::SharedPtr network_state_sub_;

  // Services
  rclcpp::Service<task_offloading_interfaces::srv::DecideOffloading>::SharedPtr decide_service_;
  rclcpp::Service<task_offloading_interfaces::srv::EstimateParameters>::SharedPtr estimate_service_;

  // Components
  std::unique_ptr<decision_maker::ParameterEstimator> estimator_;
  std::unique_ptr<task_offloading_core::DecisionModel> decision_model_;

  // Latest state
  task_offloading_interfaces::msg::SystemState last_system_state_;
  task_offloading_interfaces::msg::NetworkState last_network_state_;
  bool has_system_state_ = false;
  bool has_network_state_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionMakerNode>());
  rclcpp::shutdown();
  return 0;
}