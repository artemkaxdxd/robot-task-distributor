#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/msg/edge_station_status.hpp"
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

class StationMonitorNode : public rclcpp::Node
{
public:
  StationMonitorNode()
  : Node("station_monitor_node")
  {
    // Create publisher
    status_pub_ = this->create_publisher<task_offloading_interfaces::msg::EdgeStationStatus>(
      "/edge/status", 10);

    // Create timer
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StationMonitorNode::publishStatus, this));

    RCLCPP_INFO(this->get_logger(), "Station Monitor Node started");
  }

private:
  void publishStatus()
  {
    auto msg = task_offloading_interfaces::msg::EdgeStationStatus();
    msg.timestamp = this->now();
    
    msg.cpu_load = getCPULoad();
    msg.gpu_load = 0.0f; // Would need nvidia-smi or similar for real GPU monitoring
    msg.memory_available_mb = getAvailableMemoryMB();
    msg.queue_length = 0; // Filled by scheduler
    msg.avg_queue_time_sec = 0.0;
    msg.active_robots = 0; // Would track connected robots

    status_pub_->publish(msg);
  }

  float getCPULoad()
  {
    // Simplified CPU load reading
    static unsigned long long prev_idle = 0;
    static unsigned long long prev_total = 0;

    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
      return 0.0f;
    }

    std::string line;
    std::getline(file, line);

    std::istringstream iss(line);
    std::string cpu;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    
    iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    unsigned long long idle_time = idle + iowait;
    unsigned long long total_time = user + nice + system + idle + iowait + irq + softirq + steal;

    unsigned long long idle_delta = idle_time - prev_idle;
    unsigned long long total_delta = total_time - prev_total;

    prev_idle = idle_time;
    prev_total = total_time;

    if (total_delta == 0) {
      return 0.0f;
    }

    return 1.0f - (static_cast<float>(idle_delta) / static_cast<float>(total_delta));
  }

  uint32_t getAvailableMemoryMB()
  {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) {
      return 0;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.find("MemAvailable:") == 0) {
        std::istringstream iss(line);
        std::string key;
        uint32_t value;
        iss >> key >> value;
        return value / 1024; // Convert KB to MB
      }
    }

    return 0;
  }

  rclcpp::Publisher<task_offloading_interfaces::msg::EdgeStationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}