#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/srv/execute_task.hpp"
#include "task_offloading_interfaces/msg/edge_station_status.hpp"
#include "edge_server/task_scheduler.hpp"
#include "edge_server/edge_computation_modules.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class EdgeServerNode : public rclcpp::Node
{
public:
  EdgeServerNode()
  : Node("edge_server_node")
  {
    // Declare parameters
    this->declare_parameter("num_workers", 2);
    this->declare_parameter("status_publish_rate_hz", 1.0);

    int num_workers = this->get_parameter("num_workers").as_int();
    double status_rate = this->get_parameter("status_publish_rate_hz").as_double();

    // Initialize computation modules
    initializeModules();

    // Create task scheduler
    scheduler_ = std::make_unique<edge_server::TaskScheduler>(num_workers);

    // Create service for task execution
    execute_service_ = this->create_service<task_offloading_interfaces::srv::ExecuteTask>(
      "/edge/execute_task",
      std::bind(&EdgeServerNode::handleExecuteRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    // Create publisher for station status
    status_pub_ = this->create_publisher<task_offloading_interfaces::msg::EdgeStationStatus>(
      "/edge/status", 10);

    // Create timer to publish status
    int period_ms = static_cast<int>(1000.0 / status_rate);
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&EdgeServerNode::publishStatus, this));

    RCLCPP_INFO(this->get_logger(), 
                "Edge Server Node started with %d worker threads", num_workers);
  }

  ~EdgeServerNode()
  {
    if (scheduler_) {
      scheduler_->stop();
    }
  }

private:
  void initializeModules()
  {
    auto& registry = edge_server::EdgeModuleRegistry::getInstance();
    
    // Register edge modules (with GPU support)
    registry.registerModule("object_detection",
      std::make_shared<edge_server::EdgeObjectDetectionModule>());
    registry.registerModule("path_planning",
      std::make_shared<edge_server::EdgePathPlanningModule>());
    registry.registerModule("pointcloud_processing",
      std::make_shared<edge_server::EdgePointCloudModule>());

    RCLCPP_INFO(this->get_logger(), "Registered %d edge computation modules", 3);
  }

  void handleExecuteRequest(
    const std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Request> request,
    std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received edge execution request: ID=%lu, Type=%s, Priority=%d",
                request->task_id, request->task_type.c_str(), request->priority);

    // Create task request
    auto task = std::make_shared<edge_server::TaskRequest>();
    task->task_id = request->task_id;
    task->task_type = request->task_type;
    task->priority = request->priority;
    task->request = request;
    task->response = response;

    // Create promise/future for synchronization
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    task->callback = [promise]() {
      promise->set_value();
    };

    // Enqueue task
    scheduler_->enqueueTask(task);

    // Wait for completion
    future.wait();

    RCLCPP_INFO(this->get_logger(),
                "Task %lu completed with status %d",
                request->task_id, response->status);
  }

  void publishStatus()
  {
    auto msg = task_offloading_interfaces::msg::EdgeStationStatus();
    msg.timestamp = this->now();
    
    // Get system info (simplified)
    msg.cpu_load = 0.3f; // Simulated
    msg.gpu_load = 0.2f; // Simulated
    msg.memory_available_mb = 16000; // Simulated 16GB available
    
    msg.queue_length = scheduler_->getQueueLength();
    msg.avg_queue_time_sec = scheduler_->getAverageQueueTime();
    msg.active_robots = 1; // Simulated

    status_pub_->publish(msg);
  }

  rclcpp::Service<task_offloading_interfaces::srv::ExecuteTask>::SharedPtr execute_service_;
  rclcpp::Publisher<task_offloading_interfaces::msg::EdgeStationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  
  std::unique_ptr<edge_server::TaskScheduler> scheduler_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgeServerNode>());
  rclcpp::shutdown();
  return 0;
}