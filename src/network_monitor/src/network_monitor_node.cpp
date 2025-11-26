#include <rclcpp/rclcpp.hpp>
#include "task_offloading_interfaces/msg/network_state.hpp"
#include "network_monitor/network_monitor.hpp"
#include <chrono>

using namespace std::chrono_literals;

class NetworkMonitorNode : public rclcpp::Node
{
public:
  NetworkMonitorNode()
  : Node("network_monitor_node")
  {
    // Declare parameters
    this->declare_parameter("edge_address", "127.0.0.1");
    this->declare_parameter("edge_port", 50051);
    this->declare_parameter("publish_rate_hz", 0.5);

    std::string edge_address = this->get_parameter("edge_address").as_string();
    int edge_port = this->get_parameter("edge_port").as_int();
    double publish_rate = this->get_parameter("publish_rate_hz").as_double();

    // Create network monitor
    monitor_ = std::make_unique<network_monitor::NetworkMonitor>(edge_address, edge_port);

    // Create publisher for network state
    network_state_pub_ = this->create_publisher<task_offloading_interfaces::msg::NetworkState>(
      "/robot/network_state", 10);

    // Create timer to publish network state
    int period_ms = static_cast<int>(1000.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&NetworkMonitorNode::publishNetworkState, this));

    RCLCPP_INFO(this->get_logger(), 
                "Network Monitor Node started (edge: %s:%d, rate: %.1f Hz)",
                edge_address.c_str(), edge_port, publish_rate);
  }

private:
  void publishNetworkState()
  {
    auto metrics = monitor_->getMetrics();

    auto msg = task_offloading_interfaces::msg::NetworkState();
    msg.timestamp = this->now();
    msg.signal_strength_dbm = metrics.signal_strength_dbm;
    msg.bandwidth_up_mbps = metrics.bandwidth_up_mbps;
    msg.bandwidth_down_mbps = metrics.bandwidth_down_mbps;
    msg.latency_ms = metrics.latency_ms;
    msg.packet_loss_rate = metrics.packet_loss_rate;
    msg.edge_available = metrics.edge_available;

    network_state_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Network State: Signal=%.1f dBm, BW=%.1f/%.1f Mbps, Latency=%.1f ms, Loss=%.2f%%",
                 msg.signal_strength_dbm,
                 msg.bandwidth_up_mbps,
                 msg.bandwidth_down_mbps,
                 msg.latency_ms,
                 msg.packet_loss_rate * 100.0f);
  }

  rclcpp::Publisher<task_offloading_interfaces::msg::NetworkState>::SharedPtr network_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<network_monitor::NetworkMonitor> monitor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetworkMonitorNode>());
  rclcpp::shutdown();
  return 0;
}