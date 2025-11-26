#ifndef EDGE_SERVER__TASK_SCHEDULER_HPP_
#define EDGE_SERVER__TASK_SCHEDULER_HPP_

#include "task_offloading_interfaces/srv/execute_task.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>
#include <memory>
#include <atomic>

namespace edge_server
{

struct TaskRequest
{
  uint64_t task_id;
  std::string task_type;
  uint8_t priority;
  std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Request> request;
  std::shared_ptr<task_offloading_interfaces::srv::ExecuteTask::Response> response;
  std::function<void()> callback;
  std::chrono::steady_clock::time_point enqueue_time;
};

// Comparator for priority queue (higher priority = higher value)
struct TaskComparator
{
  bool operator()(const std::shared_ptr<TaskRequest>& a,
                  const std::shared_ptr<TaskRequest>& b) const
  {
    return a->priority < b->priority; // Min-heap, so reverse for max-heap
  }
};

class TaskScheduler
{
public:
  TaskScheduler(int num_worker_threads = 2);
  ~TaskScheduler();

  // Enqueue task for execution
  void enqueueTask(std::shared_ptr<TaskRequest> task);

  // Get current queue length
  size_t getQueueLength() const;

  // Get average queue time
  double getAverageQueueTime() const;

  // Get number of active workers
  int getActiveWorkers() const;

  // Stop scheduler
  void stop();

private:
  // Worker thread function
  void workerThread();

  // Execute a task
  void executeTask(std::shared_ptr<TaskRequest> task);

  // Priority queue for tasks
  std::priority_queue<std::shared_ptr<TaskRequest>,
                     std::vector<std::shared_ptr<TaskRequest>>,
                     TaskComparator> task_queue_;

  // Synchronization
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // Worker threads
  std::vector<std::thread> workers_;
  std::atomic<bool> stop_flag_;
  std::atomic<int> active_workers_;

  // Statistics
  std::atomic<double> total_queue_time_;
  std::atomic<int> completed_tasks_;
};

} // namespace edge_server

#endif // EDGE_SERVER__TASK_SCHEDULER_HPP_