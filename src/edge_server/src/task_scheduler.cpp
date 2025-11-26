#include "edge_server/task_scheduler.hpp"
#include "edge_server/edge_computation_modules.hpp"
#include <iostream>

namespace edge_server
{

TaskScheduler::TaskScheduler(int num_worker_threads)
: stop_flag_(false), active_workers_(0), total_queue_time_(0.0), completed_tasks_(0)
{
  // Create worker threads
  for (int i = 0; i < num_worker_threads; ++i) {
    workers_.emplace_back(&TaskScheduler::workerThread, this);
  }
}

TaskScheduler::~TaskScheduler()
{
  stop();
}

void TaskScheduler::enqueueTask(std::shared_ptr<TaskRequest> task)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  task->enqueue_time = std::chrono::steady_clock::now();
  task_queue_.push(task);
  queue_cv_.notify_one();
}

size_t TaskScheduler::getQueueLength() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return task_queue_.size();
}

double TaskScheduler::getAverageQueueTime() const
{
  int completed = completed_tasks_.load();
  if (completed == 0) {
    return 0.0;
  }
  return total_queue_time_.load() / completed;
}

int TaskScheduler::getActiveWorkers() const
{
  return active_workers_.load();
}

void TaskScheduler::stop()
{
  stop_flag_ = true;
  queue_cv_.notify_all();

  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void TaskScheduler::workerThread()
{
  while (!stop_flag_) {
    std::shared_ptr<TaskRequest> task;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return !task_queue_.empty() || stop_flag_;
      });

      if (stop_flag_ && task_queue_.empty()) {
        break;
      }

      if (!task_queue_.empty()) {
        task = task_queue_.top();
        task_queue_.pop();
      }
    }

    if (task) {
      executeTask(task);
    }
  }
}

void TaskScheduler::executeTask(std::shared_ptr<TaskRequest> task)
{
  active_workers_++;

  auto dequeue_time = std::chrono::steady_clock::now();
  auto queue_duration = std::chrono::duration_cast<std::chrono::microseconds>(
    dequeue_time - task->enqueue_time);

  // Update queue time statistics
  double queue_time_sec = queue_duration.count() / 1e6;
  // Atomic add for double using compare-and-swap loop
  double expected = total_queue_time_.load();
  while (!total_queue_time_.compare_exchange_weak(expected, expected + queue_time_sec)) {
    // expected was updated with the current value, retry
  }
  completed_tasks_++;

  auto& registry = EdgeModuleRegistry::getInstance();
  auto module = registry.getModule(task->task_type);

  if (!module) {
    task->response->status = 1;
    task->response->error_message = "Unknown task type: " + task->task_type;
    task->callback();
    active_workers_--;
    return;
  }

  auto compute_start = std::chrono::steady_clock::now();

  try {
    // Execute task
    task->response->output_data = module->execute(
      task->request->parameters, task->request->input_data);
    task->response->status = 0;

    auto compute_end = std::chrono::steady_clock::now();
    auto compute_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      compute_end - compute_start);

    // Fill metrics
    task->response->metrics.compute_time_sec = compute_duration.count() / 1e6;
    task->response->metrics.queue_time_sec = queue_time_sec;

  } catch (const std::exception& e) {
    task->response->status = 1;
    task->response->error_message = std::string("Execution error: ") + e.what();
  }

  // Invoke callback
  task->callback();

  active_workers_--;
}

} // namespace edge_server