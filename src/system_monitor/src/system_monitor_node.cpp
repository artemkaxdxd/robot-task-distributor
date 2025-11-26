#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/msg/system_state.hpp"
#include "system_monitor/system_monitor.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode()
  : Node("system_monitor_node")
  {
    // Create publisher for system state
    system_state_pub_ = this->create_publisher<task_offloading_interfaces::msg::SystemState>(
      "/robot/system_state", 10);

    // Create timer to publish system state at 1 Hz
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&SystemMonitorNode::publishSystemState, this));

    RCLCPP_INFO(this->get_logger(), "System Monitor Node started");
  }

private:
  void publishSystemState()
  {
    auto msg = task_offloading_interfaces::msg::SystemState();
    msg.timestamp = this->now();
    msg.battery_level = monitor_.getBatteryLevel();
    msg.cpu_usage = monitor_.getCpuUsage();
    msg.memory_usage = monitor_.getMemoryUsage();
    msg.cpu_temperature = monitor_.getCpuTemperature();
    msg.memory_available_mb = monitor_.getAvailableMemoryMB();

    system_state_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "System State: Battery=%.2f%%, CPU=%.1f%%, Temp=%.1f°C",
                 msg.battery_level * 100.0f,
                 msg.cpu_usage * 100.0f,
                 msg.cpu_temperature);
  }

  rclcpp::Publisher<task_offloading_interfaces::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  system_monitor::SystemMonitor monitor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}