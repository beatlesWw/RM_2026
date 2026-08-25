#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <optional>

#include "io/cboard.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/plotter.hpp"

//多线程的“指令生成器
namespace auto_aim
{
namespace multithread
{

class CommandGener
{
public:
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
    tools::Plotter & plotter, bool debug = false);

  ~CommandGener();//析构函数，用来退出

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed, const Eigen::Vector3d & gimbal_pos);

private:
  struct Input
  {
    std::list<auto_aim::Target> targets_;//目标列表
    std::chrono::steady_clock::time_point t;//时间
    // std::function<void()> decide;
    //上一行原本是注释的
    double bullet_speed;//子弹速度
    Eigen::Vector3d gimbal_pos;//云台位置
  };

  io::CBoard & cboard_;//C板
  auto_aim::Shooter & shooter_;
  auto_aim::Aimer & aimer_;
  tools::Plotter & plotter_;//引用绘图器

  std::optional<Input> latest_;//输入
  std::mutex mtx_;//互斥锁
  std::condition_variable cv_;//条件变量
  std::thread thread_;//线程
  bool stop_, debug_;//停止标志位

  void generate_command();//函数
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP