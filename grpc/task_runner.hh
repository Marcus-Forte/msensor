#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <stop_token>
#include <thread>

/**
 * @brief Single worker thread that runs posted tasks in order.
 *
 * Used to move per-client work (message conversion, filtering) off sensor
 * producer threads. Tasks still queued when the runner is destroyed are run
 * before the thread exits.
 */
class TaskRunner {
public:
  TaskRunner() : thread_([this](std::stop_token st) { run(st); }) {}
  ~TaskRunner() {
    thread_.request_stop();
    cv_.notify_all();
  }

  TaskRunner(const TaskRunner &) = delete;
  TaskRunner &operator=(const TaskRunner &) = delete;

  void post(std::function<void()> task) {
    {
      std::lock_guard lock(m_);
      queue_.push_back(std::move(task));
    }
    cv_.notify_one();
  }

private:
  void run(std::stop_token st) {
    std::unique_lock lock(m_);
    while (true) {
      cv_.wait(lock, st, [this] { return !queue_.empty(); });
      if (queue_.empty())
        return; // stop requested and nothing left to run
      auto task = std::move(queue_.front());
      queue_.pop_front();
      lock.unlock();
      task();
      lock.lock();
    }
  }

  std::mutex m_;
  std::condition_variable_any cv_;
  std::deque<std::function<void()>> queue_;
  std::jthread thread_; // last: joined first
};
