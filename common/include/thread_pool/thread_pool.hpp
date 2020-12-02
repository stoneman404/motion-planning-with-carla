
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_THREAD_POOL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_THREAD_POOL_HPP_
//#include <vector>
//#include <future>
//#include <queue>
//#include <functional>
//namespace behaviour_planner {
//class ThreadPool {
// public:
//  explicit ThreadPool(size_t pool_size = 1);
//  ~ThreadPool();
//  template<typename Func, typename ...Args>
//  std::future<typename std::result_of<Func(Args...)>::type> Enqueue(Func &&f, Args &&...args);
// private:
//  size_t pool_size_;
//  std::vector<std::thread> workers_;
//  std::queue<std::function<void()>> tasks_;
//  std::mutex mtx_;
//  std::condition_variable cv_;
//  bool stop_;
//};
//
//inline ThreadPool::ThreadPool(size_t pool_size)
//    : pool_size_(std::max(pool_size, 1lu)), stop_(false) {
//  for (size_t i = 0; i < pool_size_; ++i) {
//    workers_.emplace_back(
//        [this]() -> void {
//          while (true) {
//            std::function<void()> task;
//            {
//              // critical region
//              std::unique_lock<std::mutex> lock(this->mtx_);
//              this->cv_.wait(lock, [this]() -> bool { return this->stop_ || !this->tasks_.empty(); });
//              if (this->stop_ && this->tasks_.empty()) {
//                return;
//              }
//              task = std::move(this->tasks_.front());
//              this->tasks_.pop();
//            }
//            task();
//          }
//        }
//    );
//  }
//}
//
//inline ThreadPool::~ThreadPool() {
//  {
//    //critical region
//    std::unique_lock<std::mutex> lock(mtx_);
//    stop_ = true;
//  }
//  cv_.notify_all();
//  for (std::thread &worker : workers_) {
//    worker.join();
//  }
//}
//
//template<typename Func, typename... Args>
//std::future<typename std::result_of<Func(Args...)>::type> ThreadPool::Enqueue(Func &&f, Args &&... args) {
//
//  using return_type = typename std::result_of<Func(Args...)>::type;
//  if (stop_) {
//    return std::future<return_type>();
//  }
//  auto task_ptr = std::make_shared<std::packaged_task<return_type()>>(
//      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));
//  std::future<return_type> res = task_ptr->get_future();
//  {
//    //critical region
//    std::unique_lock<std::mutex> lock(mtx_);
//    tasks_.emplace([task_ptr]() {
//      (*task_ptr)();
//    });
//  }
//  cv_.notify_one();
//  return res;
//}
//}

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace common {

class ThreadPool {
 public:
  // If the input pool_size < 1, it will be fixed to 1.
  ThreadPool(int pool_size = 1);
  ~ThreadPool();

  // Get the size (thread number) of thread pool.
  int Size() const;

  template<class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> PushTask(
      Func &&f, Args &&... args);

 private:
  int pool_size_;
  bool shutdown_;
  std::vector<std::thread> thread_list_;
  std::queue<std::function<void()>> task_queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
};

inline ThreadPool::ThreadPool(int pool_size)
    : pool_size_(std::max(pool_size, 1)), shutdown_(false) {
  auto thread_body = [this] {
    while (true) {
      std::function<void()> task;
      {  // Critical region.
        std::unique_lock<std::mutex> lck(mtx_);
        cv_.wait(lck, [this] { return shutdown_ || !task_queue_.empty(); });
        if (shutdown_ && task_queue_.empty()) return;
        task = std::move(task_queue_.front());
        task_queue_.pop();
      }
      task();
    }
  };
  for (int i = 0; i < pool_size_; i++) {
    thread_list_.emplace_back(thread_body);
  }
}

inline ThreadPool::~ThreadPool() {
  {  // Critical region.
    std::unique_lock<std::mutex> lck(mtx_);
    shutdown_ = true;
  }
  cv_.notify_all();
  for (auto &t : thread_list_) t.join();
}

inline int ThreadPool::Size() const { return pool_size_; }

template<class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type> ThreadPool::PushTask(
    Func &&f, Args &&... args) {
  using return_type = typename std::result_of<Func(Args...)>::type;
  // If thread pool is shutdown, return an empty future object.
  if (shutdown_) return std::future<return_type>();
  auto task_ptr = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));
  {  // Critical region.
    std::unique_lock<std::mutex> lck(mtx_);
    task_queue_.emplace([task_ptr] { (*task_ptr)(); });
  }
  cv_.notify_one();
  return task_ptr->get_future();
}

}  // namespace behaviour_planner

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_COMMON_INCLUDE_THREAD_POOL_HPP_
