#ifndef NETWORK_MONITOR__NETWORK_MONITOR_HPP_
#define NETWORK_MONITOR__NETWORK_MONITOR_HPP_

#include <string>
#include <vector>
#include <chrono>
#include <deque>

namespace network_monitor
{

struct NetworkMetrics
{
  float signal_strength_dbm;
  float bandwidth_up_mbps;
  float bandwidth_down_mbps;
  float latency_ms;
  float packet_loss_rate;
  bool edge_available;
};

class NetworkMonitor
{
public:
  NetworkMonitor(const std::string& edge_address, int edge_port);
  ~NetworkMonitor() = default;

  // Get current network metrics
  NetworkMetrics getMetrics();

  // Measure latency using ICMP ping (simplified version)
  float measureLatency();

  // Estimate bandwidth by sending test packets
  void estimateBandwidth(float& upload_mbps, float& download_mbps);

  // Check if edge station is reachable
  bool isEdgeAvailable();

  // Get signal strength (simulated for now)
  float getSignalStrength() const;

private:
  std::string edge_address_;
  int edge_port_;

  // Smoothing for measurements
  std::deque<float> latency_history_;
  std::deque<float> bandwidth_up_history_;
  std::deque<float> bandwidth_down_history_;
  
  static constexpr size_t HISTORY_SIZE = 5;

  float smoothValue(std::deque<float>& history, float new_value);
  
  // Simulated measurements for testing
  float simulateLatency() const;
  float simulateBandwidth() const;
  float simulateSignalStrength() const;
};

} // namespace network_monitor

#endif // NETWORK_MONITOR__NETWORK_MONITOR_HPP_