#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
//强阻塞、无超时、无关闭语义” 的队列
/*可以理解成：一个“多个线程安全地传数据的管道”
生产者线程：read_thread（串口线程）
消费者线程：主逻辑 / 解算线程*/
namespace tools
{
template <typename T, bool PopWhenFull = false> //真正存数据的地方
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)//max_size_是最大容量 full_handler_是最大容量满时调用的的回调函数
  {
  }
//上面冒号后面是初始化
  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);//加锁，保障数据

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();//弹出——最早的被扔出去
      } else {
        full_handler_();//一个队列满时的回调函数
        return;
      }
    }

    queue_.push(value);//把一个元素放进队列里
    not_empty_condition_.notify_all();//队列现在不空了，叫醒线程
  }
  //从队列里“取出一个元素”并放到 value 里
  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);//给队列上锁

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });//等待队列不空

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    value = queue_.front();//先取值
    queue_.pop();//弹出
  }

  T pop()//弹出的函数——从队列里拿出 一个元素并把它 作为返回值 给调用者
  {
    std::unique_lock<std::mutex> lock(mutex_);//开锁

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });
    /*“如果队列是空的，我就在这里等；等到队列不空了，再继续往下执行。”*/

    T value = std::move(queue_.front());//把队列最前面的那个元素“搬出来” 存到 value 里
    queue_.pop();//把刚才那个元素从队列里删掉
    return std::move(value);//把刚才拿到的元素作为函数返回值返回
  }

  /*front是等到队列里有东西，但我只看一眼，不拿走“，即不删除元素*/
  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);//开锁

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });
    /*“如果队列是空的，我就在这里等；等到队列不空了，再继续往下执行。”*/

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();//取最后一个元素赋值value
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  { 
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();/*把队列里的所有元素清空*/
    }
    not_empty_condition_.notify_all();  // 如果其他线程正在等待队列不为空，这样可以唤醒它们
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP