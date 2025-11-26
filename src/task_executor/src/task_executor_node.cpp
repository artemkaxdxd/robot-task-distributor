#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/srv/execute_task.hpp"
#include "task_offloading_interfaces/srv/decide_offloading.hpp"
#include "task_offloading_interfaces/srv/record_execution.hpp"
#include "task_offloading_interfaces/msg/system_state.hpp"
#include "task_offloading_interfaces/msg/network_state.hpp"
#include "task_executor/computation_modules.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
  : Node("task_executor_node")
  {
    // Initialize computation modules
    initializeModules();

    // Create service for task execution
    execute_service_ = this->create_service<task_offloading_interfaces::srv::ExecuteTask>(
      "/task/execute",
      std::bind(&TaskExecutorNode::handleExecuteRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    // Create client for decision maker
    decision_client_ = this->create_client<task_offloading_interfaces::srv::DecideOffloading>(
      "/offloading/decide");

    // Create client for edge execution
    edge_client_ = this->create_client<task_offloading_interfaces::srv::ExecuteTask>(
      "/edge/execute_task");

    // Create client for recording execution metrics
    record_client_ = this->create_client<task_offloading_interfaces::srv::RecordExecution>(
      "/profile/record");

    // Subscribe to system and network state
    system_state_sub_ = this->create_subscription<task_offloading_interfaces::msg::SystemState>(
      "/robot/system_state", 10,
      std::bind(&TaskExecutorNode::systemStateCallback, this, std::placeholders::_1));

    network_state_sub_ = this->create_subscription<task_offloading_interfaces::msg::NetworkState>(
      "/robot/network_state", 10,
      std::bind(&TaskExecutorNode::networkStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Task Executor Node started");
  }

private:
  void initializeModules()
  {
    auto& registry = task_executor::ModuleRegistry::getInstance();
    
    // Register local modules (CPU only)
    registry.registerModule("object_detection",
      std::make_shared<task_executor::ObjectDetectionModule>(false));
    registry.registerModule("path_planning",
      std::make_shared<task_executor::PathPlanningModule>());
    registry.registerModule("pointcloud_processing",
      std::make_shared<task_executor::PointCloudModule>());

    RCLCPP_INFO(this->get_logger(), "Registered %d computation modules", 3);
  }

  void systemStateCallback(const task_offloading_interfaces::msg::SystemState::SharedPtr msg)
  {
    last_system_state_ = *msg;
  }

  void networkStateCallback(const task_offloading_interfaces::msg::NetworkState::SharedPtr msg)
  {
    last_network_state_ = *msg;
  }

  void handleExecuteRequest(
    const std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Request> request,
    std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Response> response)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), 
                "Received task execution request: ID=%lu, Type=%s",
                request->task_id, request->task_type.c_str());

    // Step 1: Get decision from decision maker
    auto decision_request = std::make_shared<task_offloading_interfaces::srv::DecideOffloading::Request>();
    decision_request->task_id = request->task_id;
    decision_request->task_type = request->task_type;
    decision_request->parameters = request->parameters;
    decision_request->input_data_size = request->input_data.size();
    decision_request->output_data_size = 1024; // Estimate
    decision_request->max_latency_sec = request->max_latency_sec;
    decision_request->priority = request->priority;
    decision_request->is_critical = (request->priority > 200);

    if (!decision_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Decision service not available, executing locally");
      executeLocally(request, response, start_time);
      return;
    }

    auto decision_future = decision_client_->async_send_request(decision_request);
    
    // Wait for decision with timeout
    if (decision_future.wait_for(100ms) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Decision timeout, executing locally");
      executeLocally(request, response, start_time);
      return;
    }

    auto decision_response = decision_future.get();

    // Step 2: Execute based on decision
    if (decision_response->execute_remotely) {
      RCLCPP_INFO(this->get_logger(), "Executing task %lu remotely", request->task_id);
      executeRemotely(request, response, start_time, decision_response);
    } else {
      RCLCPP_INFO(this->get_logger(), "Executing task %lu locally", request->task_id);
      executeLocally(request, response, start_time);
    }
  }

  void executeLocally(
    const std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Request> request,
    std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Response> response,
    std::chrono::high_resolution_clock::time_point start_time)
  {
    auto& registry = task_executor::ModuleRegistry::getInstance();
    
    // Get computation module
    auto module = registry.getModule(request->task_type);
    if (!module) {
      RCLCPP_ERROR(this->get_logger(), "Unknown task type: %s", request->task_type.c_str());
      response->status = 1; // Error
      response->error_message = "Unknown task type: " + request->task_type;
      return;
    }

    auto compute_start = std::chrono::high_resolution_clock::now();

    try {
      // Execute the task
      response->output_data = module->execute(request->parameters, request->input_data);
      response->status = 0; // Success

      auto compute_end = std::chrono::high_resolution_clock::now();
      auto compute_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        compute_end - compute_start);

      // Fill metrics
      response->metrics.compute_time_sec = compute_duration.count() / 1e6;
      response->metrics.queue_time_sec = 0.0;
      response->metrics.transfer_in_time_sec = 0.0;
      response->metrics.transfer_out_time_sec = 0.0;

      auto end_time = std::chrono::high_resolution_clock::now();
      auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);

      RCLCPP_INFO(this->get_logger(),
                  "Task %lu completed locally in %.3f ms",
                  request->task_id, total_duration.count() / 1000.0);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Local execution failed: %s", e.what());
      response->status = 1;
      response->error_message = std::string("Execution error: ") + e.what();
    }
  }

  void executeRemotely(
    const std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Request> request,
    std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Response> response,
    std::chrono::high_resolution_clock::time_point start_time,
    const std::shared_ptr<task_offloading_interfaces::srv::DecideOffloading::Response> decision)
  {
    // Check if edge service is available
    if (!edge_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), 
                  "Edge service not available, falling back to local execution");
      executeLocally(request, response, start_time);
      return;
    }

    // Forward request to edge server
    auto edge_future = edge_client_->async_send_request(request);

    // Wait for response with timeout
    double timeout_sec = std::max(request->max_latency_sec, 5.0);
    auto timeout = std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000));

    if (edge_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Edge execution timeout for task %lu, falling back to local",
                   request->task_id);
      executeLocally(request, response, start_time);
      return;
    }

    auto edge_response = edge_future.get();

    // Check if edge execution was successful
    if (edge_response->status != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Edge execution failed: %s, falling back to local",
                  edge_response->error_message.c_str());
      executeLocally(request, response, start_time);
      return;
    }

    // Copy response from edge
    *response = *edge_response;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);

    RCLCPP_INFO(this->get_logger(),
                "Task %lu completed remotely in %.3f ms (compute: %.3f ms)",
                request->task_id,
                total_duration.count() / 1000.0,
                response->metrics.compute_time_sec * 1000.0);
  }

  // Service and clients
  rclcpp::Service<task_offloading_interfaces::srv::ExecuteTask>::SharedPtr execute_service_;
  rclcpp::Client<task_offloading_interfaces::srv::DecideOffloading>::SharedPtr decision_client_;
  rclcpp::Client<task_offloading_interfaces::srv::ExecuteTask>::SharedPtr edge_client_;
  rclcpp::Client<task_offloading_interfaces::srv::RecordExecution>::SharedPtr record_client_;

  // Subscribers
  rclcpp::Subscription<task_offloading_interfaces::msg::SystemState>::SharedPtr system_state_sub_;
  rclcpp::Subscription<task_offloading_interfaces::msg::NetworkState>::SharedPtr network_state_sub_;

  // Latest state
  task_offloading_interfaces::msg::SystemState last_system_state_;
  task_offloading_interfaces::msg::NetworkState last_network_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskExecutorNode>());
  rclcpp::shutdown();
  return 0;
}