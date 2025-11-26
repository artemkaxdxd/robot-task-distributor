#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/srv/execute_task.hpp"
#include "task_offloading_interfaces/msg/task_parameters.hpp"
#include <random>
#include <chrono>

using namespace std::chrono_literals;

class TaskGeneratorNode : public rclcpp::Node
{
public:
  TaskGeneratorNode()
  : Node("task_generator_node"), task_counter_(0)
  {
    // Declare parameters
    this->declare_parameter("generation_rate_hz", 0.2);
    this->declare_parameter("task_types", std::vector<std::string>{
      "object_detection", "path_planning", "pointcloud_processing"
    });

    double rate = this->get_parameter("generation_rate_hz").as_double();
    task_types_ = this->get_parameter("task_types").as_string_array();

    // Create client for task execution
    execute_client_ = this->create_client<task_offloading_interfaces::srv::ExecuteTask>(
      "/task/execute");

    // Create timer to generate tasks
    int period_ms = static_cast<int>(1000.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&TaskGeneratorNode::generateTask, this));

    RCLCPP_INFO(this->get_logger(), 
                "Task Generator Node started (rate: %.2f Hz)", rate);
  }

private:
  void generateTask()
  {
    if (!execute_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(), "Task execution service not available");
      return;
    }

    // Randomly select task type
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> type_dist(0, task_types_.size() - 1);
    std::string task_type = task_types_[type_dist(gen)];

    // Create request
    auto request = std::make_shared<task_offloading_interfaces::srv::ExecuteTask::Request>();
    request->task_id = ++task_counter_;
    request->task_type = task_type;
    request->priority = generatePriority();

    // Fill task-specific parameters
    fillParameters(task_type, request->parameters);

    // Generate dummy input data
    request->input_data = generateInputData(task_type);

    // Set constraints
    request->max_latency_sec = 2.0; // 2 seconds max latency

    RCLCPP_INFO(this->get_logger(),
                "Generating task %lu: type=%s, priority=%d",
                request->task_id, task_type.c_str(), request->priority);

    // Send request asynchronously
    uint64_t task_id = request->task_id;
    auto future = execute_client_->async_send_request(request,
      [this, task_id, task_type](
        rclcpp::Client<task_offloading_interfaces::srv::ExecuteTask>::SharedFuture future)
      {
        this->handleResponse(future, task_id, task_type);
      });
  }

  void handleResponse(
    rclcpp::Client<task_offloading_interfaces::srv::ExecuteTask>::SharedFuture future,
    uint64_t task_id,
    const std::string& task_type)
  {
    try {
      auto response = future.get();
      
      if (response->status == 0) {
        double total_time = response->metrics.compute_time_sec +
                          response->metrics.queue_time_sec +
                          response->metrics.transfer_in_time_sec +
                          response->metrics.transfer_out_time_sec;

        RCLCPP_INFO(this->get_logger(),
                    "Task %lu (%s) completed successfully in %.3f sec",
                    task_id, task_type.c_str(), total_time);

        RCLCPP_INFO(this->get_logger(),
                    "  Breakdown: queue=%.3fs, compute=%.3fs, transfer=%.3fs",
                    response->metrics.queue_time_sec,
                    response->metrics.compute_time_sec,
                    response->metrics.transfer_in_time_sec + response->metrics.transfer_out_time_sec);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Task %lu (%s) failed: %s",
                     task_id, task_type.c_str(), response->error_message.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Exception in task %lu response: %s",
                   task_id, e.what());
    }
  }

  uint8_t generatePriority()
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(50, 200);
    return static_cast<uint8_t>(dist(gen));
  }

  void fillParameters(const std::string& task_type,
                     task_offloading_interfaces::msg::TaskParameters& params)
  {
    std::random_device rd;
    std::mt19937 gen(rd());

    if (task_type == "object_detection") {
      // Random image sizes
      std::vector<std::pair<int, int>> sizes = {
        {640, 480}, {1280, 720}, {1920, 1080}, {320, 240}
      };
      std::uniform_int_distribution<> size_dist(0, sizes.size() - 1);
      auto size = sizes[size_dist(gen)];

      params.image_width = size.first;
      params.image_height = size.second;
      params.image_encoding = "rgb8";
      params.confidence_threshold = 0.5f;
      params.target_classes = {"person", "car", "dog", "cat"};

      // Estimate complexity
      double pixels = params.image_width * params.image_height;
      params.complexity_estimate = (pixels / (640.0 * 480.0)) * 10.0;

    } else if (task_type == "path_planning") {
      params.complexity_estimate = 5.0;

    } else if (task_type == "pointcloud_processing") {
      // Random number of points
      std::uniform_int_distribution<> points_dist(50000, 200000);
      params.num_points = points_dist(gen);
      params.voxel_size = 0.05f;
      
      params.complexity_estimate = (params.num_points / 100000.0) * 10.0;
    }
  }

  std::vector<uint8_t> generateInputData(const std::string& task_type)
  {
    // Generate dummy input data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 255);

    size_t data_size;
    if (task_type == "object_detection") {
      data_size = 640 * 480 * 3; // ~900KB for VGA RGB image
    } else if (task_type == "path_planning") {
      data_size = 1024 * 100; // 100KB map data
    } else if (task_type == "pointcloud_processing") {
      data_size = 100000 * 12; // ~1.2MB for 100k points (x,y,z as float)
    } else {
      data_size = 1024; // 1KB default
    }

    std::vector<uint8_t> data(data_size);
    for (size_t i = 0; i < data_size; ++i) {
      data[i] = static_cast<uint8_t>(dist(gen));
    }

    return data;
  }

  rclcpp::Client<task_offloading_interfaces::srv::ExecuteTask>::SharedPtr execute_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::vector<std::string> task_types_;
  std::atomic<uint64_t> task_counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}