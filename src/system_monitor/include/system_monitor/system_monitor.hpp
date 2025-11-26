#ifndef SYSTEM_MONITOR__SYSTEM_MONITOR_HPP_
#define SYSTEM_MONITOR__SYSTEM_MONITOR_HPP_

#include <string>
#include <fstream>
#include <sstream>

namespace system_monitor
{

class SystemMonitor
{
public:
  SystemMonitor();
  ~SystemMonitor() = default;

  // Get current battery level (0.0 - 1.0)
  float getBatteryLevel() const;

  // Get CPU usage (0.0 - 1.0)
  float getCpuUsage();

  // Get memory usage (0.0 - 1.0)
  float getMemoryUsage() const;

  // Get CPU temperature in Celsius
  float getCpuTemperature() const;

  // Get available memory in MB
  uint32_t getAvailableMemoryMB() const;

private:
  // Read battery information from sysfs
  float readBatteryFromSysfs() const;

  // Parse /proc/stat for CPU usage
  float calculateCpuUsage();

  // Parse /proc/meminfo for memory information
  void readMemoryInfo(uint32_t& total_mb, uint32_t& available_mb) const;

  // Read temperature from thermal zone
  float readTemperatureFromSysfs() const;

  // Previous CPU stats for usage calculation
  unsigned long long prev_idle_;
  unsigned long long prev_total_;
  bool first_cpu_read_;
};

} // namespace system_monitor

#endif // SYSTEM_MONITOR__SYSTEM_MONITOR_HPP_