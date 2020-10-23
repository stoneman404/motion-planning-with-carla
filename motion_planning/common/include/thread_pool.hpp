
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_COMMON_INCLUDE_THREAD_POOL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_COMMON_INCLUDE_THREAD_POOL_HPP_
#include <vector>
#include <future>
#include <queue>
#include <functional>
namespace planning {
class ThreadPool {
 public:
  explicit ThreadPool(size_t pool_size = 1);
  ~ThreadPool();
  template<typename Func, typename ...Args>
  std::future<typename std::result_of<Func(Args...)>::type> Enqueue(Func &&f, Args &&...args);
 private:
  size_t pool_size_;
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool stop_;
};

inline ThreadPool::ThreadPool(size_t pool_size)
    : pool_size_(std::max(pool_size, 1lu)), stop_(false) {
  for (size_t i = 0; i < pool_size_; ++i) {
    workers_.emplace_back(
        [this]() -> void {
          while (true) {
            std::function<void()> task;
            {
              // critical region
              std::unique_lock<std::mutex> lock(this->mtx_);
              this->cv_.wait(lock, [this]() -> bool { return this->stop_ || !this->tasks_.empty(); });
              if (this->stop_ && this->tasks_.empty()) {
                return;
              }
              task = std::move(this->tasks_.front());
              this->tasks_.pop();
            }
            task();
          }
        }
    );
  }
}

inline ThreadPool::~ThreadPool() {
  {
    //critical region
    std::unique_lock<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  for (std::thread &worker : workers_) {
    worker.join();
  }
}

template<typename Func, typename... Args>
std::future<typename std::result_of<Func(Args...)>::type> ThreadPool::Enqueue(Func &&f, Args &&... args) {

  using return_type = typename std::result_of<Func(Args...)>::type;
  if (stop_) {
    return std::future<return_type>();
  }
  auto task_ptr = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));
  std::future<return_type> res = task_ptr->get_future();
  {
    //critical region
    std::unique_lock<std::mutex> lock(mtx_);
    tasks_.emplace([task_ptr]() {
      (*task_ptr)();
    });
  }
  cv_.notify_one();
  return res;
}
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_COMMON_INCLUDE_THREAD_POOL_HPP_
