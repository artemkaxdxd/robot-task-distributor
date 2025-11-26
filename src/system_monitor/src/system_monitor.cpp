#include "system_monitor/system_monitor.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <array>
#include <memory>
#include <stdexcept>
#include <vector>

#ifdef __APPLE__
#include <sys/sysctl.h>
#include <mach/mach.h>
#include <IOKit/ps/IOPowerSources.h>
#include <IOKit/ps/IOPSKeys.h>
#endif

namespace system_monitor
{

SystemMonitor::SystemMonitor()
: prev_idle_(0), prev_total_(0), first_cpu_read_(true)
{
}

#ifdef __APPLE__

// macOS implementation
float SystemMonitor::getBatteryLevel() const
{
  CFTypeRef blob = IOPSCopyPowerSourcesInfo();
  if (!blob) {
    return 1.0f; // AC powered or no battery
  }

  CFArrayRef sources = IOPSCopyPowerSourcesList(blob);
  if (!sources) {
    CFRelease(blob);
    return 1.0f;
  }

  CFIndex count = CFArrayGetCount(sources);
  float capacity = 1.0f;

  for (CFIndex i = 0; i < count; i++) {
    CFDictionaryRef source = IOPSGetPowerSourceDescription(blob, CFArrayGetValueAtIndex(sources, i));
    if (!source) continue;

    CFNumberRef current_capacity_ref = (CFNumberRef)CFDictionaryGetValue(source, CFSTR(kIOPSCurrentCapacityKey));
    CFNumberRef max_capacity_ref = (CFNumberRef)CFDictionaryGetValue(source, CFSTR(kIOPSMaxCapacityKey));

    int current_capacity, max_capacity;
    if (current_capacity_ref && max_capacity_ref) {
      CFNumberGetValue(current_capacity_ref, kCFNumberIntType, &current_capacity);
      CFNumberGetValue(max_capacity_ref, kCFNumberIntType, &max_capacity);
      
      if (max_capacity > 0) {
        capacity = static_cast<float>(current_capacity) / static_cast<float>(max_capacity);
        break;
      }
    }
  }

  CFRelease(sources);
  CFRelease(blob);
  return capacity;
}

float SystemMonitor::getCpuUsage()
{
  host_cpu_load_info_data_t cpuinfo;
  mach_msg_type_number_t count = HOST_CPU_LOAD_INFO_COUNT;
  
  if (host_statistics(mach_host_self(), HOST_CPU_LOAD_INFO,
                     (host_info_t)&cpuinfo, &count) != KERN_SUCCESS) {
    return 0.0f;
  }

  unsigned long long total_ticks = 0;
  for (int i = 0; i < CPU_STATE_MAX; i++) {
    total_ticks += cpuinfo.cpu_ticks[i];
  }

  unsigned long long idle_ticks = cpuinfo.cpu_ticks[CPU_STATE_IDLE];

  if (first_cpu_read_) {
    prev_idle_ = idle_ticks;
    prev_total_ = total_ticks;
    first_cpu_read_ = false;
    return 0.0f;
  }

  unsigned long long idle_delta = idle_ticks - prev_idle_;
  unsigned long long total_delta = total_ticks - prev_total_;

  prev_idle_ = idle_ticks;
  prev_total_ = total_ticks;

  if (total_delta == 0) {
    return 0.0f;
  }

  float usage = 1.0f - (static_cast<float>(idle_delta) / static_cast<float>(total_delta));
  return std::max(0.0f, std::min(1.0f, usage));
}

float SystemMonitor::getMemoryUsage() const
{
  uint32_t total_mb, available_mb;
  readMemoryInfo(total_mb, available_mb);
  
  if (total_mb == 0) {
    return 0.0f;
  }

  return 1.0f - (static_cast<float>(available_mb) / static_cast<float>(total_mb));
}

float SystemMonitor::getCpuTemperature() const
{
  // On macOS, getting CPU temperature requires SMC (System Management Controller) access
  // which is complex and requires additional frameworks
  // For now, simulate temperature
  return 55.0f; // Simulated 55°C for testing
}

uint32_t SystemMonitor::getAvailableMemoryMB() const
{
  uint32_t total_mb, available_mb;
  readMemoryInfo(total_mb, available_mb);
  return available_mb;
}

void SystemMonitor::readMemoryInfo(uint32_t& total_mb, uint32_t& available_mb) const
{
  int mib[2];
  int64_t physical_memory;
  size_t length;

  // Get total physical memory
  mib[0] = CTL_HW;
  mib[1] = HW_MEMSIZE;
  length = sizeof(int64_t);
  sysctl(mib, 2, &physical_memory, &length, NULL, 0);

  // Get VM statistics for available memory
  vm_size_t page_size;
  vm_statistics64_data_t vm_stats;
  mach_msg_type_number_t count = sizeof(vm_stats) / sizeof(natural_t);
  mach_port_t mach_port = mach_host_self();

  host_page_size(mach_port, &page_size);

  if (host_statistics64(mach_port, HOST_VM_INFO64,
                       (host_info64_t)&vm_stats, &count) == KERN_SUCCESS) {
    int64_t free_memory = (int64_t)vm_stats.free_count * (int64_t)page_size;
    int64_t inactive_memory = (int64_t)vm_stats.inactive_count * (int64_t)page_size;
    int64_t available_memory = free_memory + inactive_memory;

    total_mb = physical_memory / (1024 * 1024);
    available_mb = available_memory / (1024 * 1024);
  } else {
    total_mb = physical_memory / (1024 * 1024);
    available_mb = total_mb / 2; // Fallback estimate
  }
}

#else

// Linux implementation
float SystemMonitor::getBatteryLevel() const
{
  // Try to read from common battery paths
  const std::vector<std::string> battery_paths = {
    "/sys/class/power_supply/BAT0/capacity",
    "/sys/class/power_supply/BAT1/capacity",
    "/sys/class/power_supply/battery/capacity"
  };

  for (const auto& path : battery_paths) {
    std::ifstream file(path);
    if (file.is_open()) {
      int capacity;
      file >> capacity;
      return capacity / 100.0f;
    }
  }

  // If no battery found, simulate for testing
  return 0.8f; // Simulated 80% for testing
}

float SystemMonitor::getCpuUsage()
{
  std::ifstream file("/proc/stat");
  if (!file.is_open()) {
    std::cerr << "Failed to open /proc/stat" << std::endl;
    return 0.0f;
  }

  std::string line;
  std::getline(file, line);
  file.close();

  std::istringstream iss(line);
  std::string cpu;
  unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
  
  iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

  unsigned long long idle_time = idle + iowait;
  unsigned long long total_time = user + nice + system + idle + iowait + irq + softirq + steal;

  if (first_cpu_read_) {
    prev_idle_ = idle_time;
    prev_total_ = total_time;
    first_cpu_read_ = false;
    return 0.0f;
  }

  unsigned long long idle_delta = idle_time - prev_idle_;
  unsigned long long total_delta = total_time - prev_total_;

  prev_idle_ = idle_time;
  prev_total_ = total_time;

  if (total_delta == 0) {
    return 0.0f;
  }

  float usage = 1.0f - (static_cast<float>(idle_delta) / static_cast<float>(total_delta));
  return std::max(0.0f, std::min(1.0f, usage));
}

float SystemMonitor::getMemoryUsage() const
{
  uint32_t total_mb, available_mb;
  readMemoryInfo(total_mb, available_mb);
  
  if (total_mb == 0) {
    return 0.0f;
  }

  return 1.0f - (static_cast<float>(available_mb) / static_cast<float>(total_mb));
}

float SystemMonitor::getCpuTemperature() const
{
  // Try common thermal zone paths
  const std::vector<std::string> temp_paths = {
    "/sys/class/thermal/thermal_zone0/temp",
    "/sys/class/thermal/thermal_zone1/temp"
  };

  for (const auto& path : temp_paths) {
    std::ifstream file(path);
    if (file.is_open()) {
      int temp_millidegrees;
      file >> temp_millidegrees;
      return temp_millidegrees / 1000.0f;
    }
  }

  // Simulate temperature if not available
  return 55.0f; // Simulated 55°C for testing
}

uint32_t SystemMonitor::getAvailableMemoryMB() const
{
  uint32_t total_mb, available_mb;
  readMemoryInfo(total_mb, available_mb);
  return available_mb;
}

void SystemMonitor::readMemoryInfo(uint32_t& total_mb, uint32_t& available_mb) const
{
  std::ifstream file("/proc/meminfo");
  if (!file.is_open()) {
    total_mb = 0;
    available_mb = 0;
    return;
  }

  std::string line;
  uint32_t mem_total_kb = 0;
  uint32_t mem_available_kb = 0;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string key;
    uint32_t value;
    std::string unit;

    iss >> key >> value >> unit;

    if (key == "MemTotal:") {
      mem_total_kb = value;
    } else if (key == "MemAvailable:") {
      mem_available_kb = value;
    }

    if (mem_total_kb > 0 && mem_available_kb > 0) {
      break;
    }
  }

  total_mb = mem_total_kb / 1024;
  available_mb = mem_available_kb / 1024;
}

#endif

} // namespace system_monitor