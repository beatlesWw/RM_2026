#include "commandgener.hpp"

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
  tools::Plotter & plotter, bool debug)
: shooter_(shooter), aimer_(aimer), cboard_(cboard), plotter_(plotter), stop_(false), debug_(debug)
{
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;//退出线程
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();//开始运行时间为t0
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);//互斥锁
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;//0.2s内的数据才有识别利用的价值，否则输出为空
    }
    if (input) {
      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);//计算瞄准
      command.shoot = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);//是否开火
      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));//计算水平距离
      cboard_.send(command);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    }//与前面承接，如果拿到了有效输出，就计算云台瞄准指令是否开火，把指令发给下位机，并在调试模式下把关键数据画图/输出。

    std::this_thread::sleep_for(std::chrono::milliseconds(5));  //approximately 原来500Hz大概休眠500赫兹 现在下位机5ms发一次
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));  //approximately 500Hz大概休眠500赫兹
  }
}

}  // namespace multithread

}  // namespace auto_aim