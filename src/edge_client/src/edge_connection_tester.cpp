#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/srv/execute_task.hpp"
#include "task_offloading_interfaces/msg/edge_station_status.hpp"
#include <chrono>

using namespace std::chrono_literals;

class EdgeConnectionTester : public rclcpp::Node
{
public:
  EdgeConnectionTester()
  : Node("edge_connection_tester")
  {
    // Create client for edge execution
    edge_client_ = this->create_client<task_offloading_interfaces::srv::ExecuteTask>(
      "/edge/execute_task");

    // Subscribe to edge status
    status_sub_ = this->create_subscription<task_offloading_interfaces::msg::EdgeStationStatus>(
      "/edge/status", 10,
      std::bind(&EdgeConnectionTester::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Edge Connection Tester started");
    RCLCPP_INFO(this->get_logger(), "Waiting for edge station...");

    // Create timer to check connection
    check_timer_ = this->create_wall_timer(
      2000ms, std::bind(&EdgeConnectionTester::checkConnection, this));

    // Create timer for sending test tasks
    test_timer_ = this->create_wall_timer(
      10000ms, std::bind(&EdgeConnectionTester::sendTestTask, this));
  }

private:
  void statusCallback(const task_offloading_interfaces::msg::EdgeStationStatus::SharedPtr msg)
  {
    last_status_time_ = this->now();
    edge_available_ = true;

    if (!status_printed_) {
      RCLCPP_INFO(this->get_logger(), 
                  "✓ Edge station is online and publishing status");
      status_printed_ = true;
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "Edge Status: CPU=%.1f%%, Queue=%d, AvgWait=%.3fs",
                 msg->cpu_load * 100.0f,
                 msg->queue_length,
                 msg->avg_queue_time_sec);
  }

  void checkConnection()
  {
    // Check if service is available
    bool service_available = edge_client_->wait_for_service(0s);

    // Check if status messages are being received
    auto now = this->now();
    auto time_since_status = (now - last_status_time_).seconds();
    bool status_recent = time_since_status < 5.0;

    if (service_available && !service_printed_) {
      RCLCPP_INFO(this->get_logger(), 
                  "✓ Edge execution service is available");
      service_printed_ = true;
    } else if (!service_available && service_printed_) {
      RCLCPP_WARN(this->get_logger(),
                  "✗ Edge execution service is NOT available");
      service_printed_ = false;
    }

    if (!service_available || !status_recent) {
      edge_available_ = false;
    }

    // Print summary
    if (edge_available_) {
      RCLCPP_INFO(this->get_logger(),
                  "Connection status: ✓ CONNECTED");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Connection status: ✗ DISCONNECTED (service=%s, status=%s)",
                  service_available ? "OK" : "UNAVAILABLE",
                  status_recent ? "OK" : "TIMEOUT");
    }
  }

  void sendTestTask()
  {
    if (!edge_available_) {
      RCLCPP_WARN(this->get_logger(), 
                  "Skipping test task - edge not available");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending test task to edge...");

    auto request = std::make_shared<task_offloading_interfaces::srv::ExecuteTask::Request>();
    request->task_id = test_counter_++;
    request->task_type = "object_detection";
    request->priority = 100;
    request->max_latency_sec = 5.0;

    // Fill parameters
    request->parameters.image_width = 640;
    request->parameters.image_height = 480;
    request->parameters.image_encoding = "rgb8";
    request->parameters.confidence_threshold = 0.5f;
    request->parameters.complexity_estimate = 10.0;

    // Dummy data
    request->input_data.resize(1024);

    auto start_time = std::chrono::steady_clock::now();

    auto future = edge_client_->async_send_request(request,
      [this, start_time](rclcpp::Client<task_offloading_interfaces::srv::ExecuteTask>::SharedFuture future) {
        try {
          auto response = future.get();
          auto end_time = std::chrono::steady_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);

          if (response->status == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "✓ Test task completed successfully in %ld ms",
                        duration.count());
            RCLCPP_INFO(this->get_logger(),
                        "  Metrics: queue=%.3fs, compute=%.3fs",
                        response->metrics.queue_time_sec,
                        response->metrics.compute_time_sec);
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "✗ Test task failed: %s",
                         response->error_message.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(),
                       "✗ Test task exception: %s", e.what());
        }
      });
  }

  rclcpp::Client<task_offloading_interfaces::srv::ExecuteTask>::SharedPtr edge_client_;
  rclcpp::Subscription<task_offloading_interfaces::msg::EdgeStationStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr test_timer_;

  rclcpp::Time last_status_time_;
  bool edge_available_ = false;
  bool service_printed_ = false;
  bool status_printed_ = false;
  uint64_t test_counter_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgeConnectionTester>());
  rclcpp::shutdown();
  return 0;
}
