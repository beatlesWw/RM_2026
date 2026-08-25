#ifndef TOOLS__THREAD_POOL_HPP
#define TOOLS__THREAD_POOL_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/logger.hpp"

namespace tools
{
//帧数据包的结构体，用来在线程间传递一帧图像及其关联信息。
struct Frame
{
  int id;//帧序号
  cv::Mat img;//图像
  std::chrono::steady_clock::time_point t;//时间戳
  Eigen::Quaterniond q;//四元数
  std::list<auto_aim::Armor> armors;//装甲板列表
};

inline std::vector<auto_aim::YOLO> create_yolo11s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO> yolo11s;
  for (int i = 0; i < numebr; i++) {
    yolo11s.push_back(auto_aim::YOLO(config_path, debug));
  }
  return yolo11s;//批量创建YOLO对象
}

inline std::vector<auto_aim::YOLO> create_yolov8s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO> yolov8s;
  for (int i = 0; i < numebr; i++) {
    yolov8s.push_back(auto_aim::YOLO(config_path, debug));
  }
  return yolov8s;
}
//按照frame.id严格顺序出队
class OrderedQueue
{
public:
  OrderedQueue() : current_id_(1) {} //current_id_：当前“期待的下一个 id”。构造时是 1。
  ~OrderedQueue()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);//锁

      main_queue_ = std::queue<tools::Frame>();//真正可以出队的队列
      buffer_.clear();//乱序逻辑
      current_id_ = 0;//当前期待的下一个 id
    }
    tools::logger()->info("OrderedQueue destroyed, queue and buffer cleared.");
  }

  void enqueue(const tools::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (item.id < current_id_) {
      tools::logger()->warn("small id");
      return;//过期的直接丢弃
    }

    if (item.id == current_id_) {
      main_queue_.push(item);//真正可以出队的队列
      current_id_++;

      auto it = buffer_.find(current_id_);//如果有刚好等于新的current_id_的buffer就进入
      while (it != buffer_.end()) {
        main_queue_.push(it->second);//将buffer中的帧加入到main_queue_中
        buffer_.erase(it);//防止重复，再buffer中删除它
        current_id_++;
        it = buffer_.find(current_id_);//如果 buffer 中存在下一帧（id = current_id_），就继续循环。
      }

      if (main_queue_.size() >= 1) {
        cond_var_.notify_one();//当你成功把至少一帧放进 main_queue 时，才会唤醒正在等待队列的线程（dequeue 的线程）
      }
    } else {
      buffer_[item.id] = item;//将item加入buffer
    }
  }
  /*阻塞出队*/
  tools::Frame dequeue()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    cond_var_.wait(lock, [this]() { return !main_queue_.empty(); });//等待

    tools::Frame item = main_queue_.front();
    main_queue_.pop();
    return item;
  }

  // 不会阻塞队列
  bool try_dequeue(tools::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (main_queue_.empty()) {
      return false;
    }
    item = main_queue_.front();
    main_queue_.pop();
    return true;
  }

  size_t get_size() { return main_queue_.size() + buffer_.size(); }

private:
  std::queue<tools::Frame> main_queue_;
  std::unordered_map<int, tools::Frame> buffer_;
  int current_id_;
  std::mutex mutex_;
  std::condition_variable cond_var_;
};

class ThreadPool
{
public:
  ThreadPool(size_t num_threads) : stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) {
              return;
            }
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
    }
  }

  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
      tasks = std::queue<std::function<void()>>();
    }
    condition.notify_all();
    for (std::thread & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  // 添加任务到任务队列
  template <class F>
  void enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();
  }

private:
  std::vector<std::thread> workers;         // 工作线程
  std::queue<std::function<void()>> tasks;  // 任务队列
  std::mutex queue_mutex;                   // 任务队列互斥锁
  std::condition_variable condition;        // 条件变量，用于等待任务
  bool stop;                                // 是否停止线程池
};
}  // namespace tools

#endif  // TOOLS__THREAD_POOL_HPP
