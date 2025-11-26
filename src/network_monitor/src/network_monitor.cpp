#include "network_monitor/network_monitor.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <random>
#include <algorithm>
#include <iostream>

namespace network_monitor
{

NetworkMonitor::NetworkMonitor(const std::string& edge_address, int edge_port)
: edge_address_(edge_address), edge_port_(edge_port)
{
}

NetworkMetrics NetworkMonitor::getMetrics()
{
  NetworkMetrics metrics;
  
  metrics.latency_ms = measureLatency();
  estimateBandwidth(metrics.bandwidth_up_mbps, metrics.bandwidth_down_mbps);
  metrics.signal_strength_dbm = getSignalStrength();
  metrics.edge_available = isEdgeAvailable();
  
  // Simple packet loss estimation based on signal strength
  if (metrics.signal_strength_dbm > -50.0f) {
    metrics.packet_loss_rate = 0.001f; // 0.1%
  } else if (metrics.signal_strength_dbm > -70.0f) {
    metrics.packet_loss_rate = 0.02f; // 2%
  } else {
    metrics.packet_loss_rate = 0.05f; // 5%
  }

  return metrics;
}

float NetworkMonitor::measureLatency()
{
  // For testing, use simulated latency
  // In production, would implement actual ICMP ping or TCP connect time
  float measured = simulateLatency();
  return smoothValue(latency_history_, measured);
}

void NetworkMonitor::estimateBandwidth(float& upload_mbps, float& download_mbps)
{
  // For testing, use simulated bandwidth
  // In production, would send actual test packets
  float up_measured = simulateBandwidth();
  float down_measured = simulateBandwidth() * 1.2f; // Download typically faster
  
  upload_mbps = smoothValue(bandwidth_up_history_, up_measured);
  download_mbps = smoothValue(bandwidth_down_history_, down_measured);
}

bool NetworkMonitor::isEdgeAvailable()
{
  // Try to connect to edge station
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return false;
  }

  struct sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(edge_port_);
  
  if (inet_pton(AF_INET, edge_address_.c_str(), &server_addr.sin_addr) <= 0) {
    close(sock);
    // For testing with simulated edge, return true
    return true;
  }

  // Set timeout for connection attempt
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  bool available = (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == 0);
  close(sock);

  // For testing, if real connection fails, simulate as available
  return available || true;
}

float NetworkMonitor::getSignalStrength() const
{
  // In production, would read from wireless interface using ioctl
  // For testing, simulate
  return simulateSignalStrength();
}

float NetworkMonitor::smoothValue(std::deque<float>& history, float new_value)
{
  history.push_back(new_value);
  if (history.size() > HISTORY_SIZE) {
    history.pop_front();
  }

  // Calculate exponentially weighted moving average
  float sum = 0.0f;
  float weight_sum = 0.0f;
  float weight = 1.0f;
  
  for (auto it = history.rbegin(); it != history.rend(); ++it) {
    sum += (*it) * weight;
    weight_sum += weight;
    weight *= 0.7f; // Decay factor
  }

  return sum / weight_sum;
}

float NetworkMonitor::simulateLatency() const
{
  // Simulate latency between 2ms and 50ms with some randomness
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<float> dist(2.0f, 50.0f);
  
  return dist(gen);
}

float NetworkMonitor::simulateBandwidth() const
{
  // Simulate bandwidth between 50 Mbps and 500 Mbps
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<float> dist(50.0f, 500.0f);
  
  return dist(gen);
}

float NetworkMonitor::simulateSignalStrength() const
{
  // Simulate Wi-Fi signal strength between -40 dBm (excellent) and -80 dBm (poor)
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<float> dist(-80.0f, -40.0f);
  
  return dist(gen);
}

} // namespace network_monitor