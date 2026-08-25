#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
//这是标准的预测自瞄主程序——有模型预测轨迹
/*while (!quit)：局部退出控制
quit 通常是一个 局部变量，只在当前线程/当前模块里有效。它通常用于：控制一个线程是否停止、控制某个功能模块是否退出、作为“本线程自己的退出开关
while (!exiter.exit())：全局退出机制
exiter 是你们项目里统一的退出控制器（工具类）。
它通常会：监听 按键 q 监听 Ctrl+C 监听 系统信号 监听 外部关闭请求。 */
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  std::atomic<bool> quit = false;//用于通知规划线程退出

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};//初始化
  auto last_mode{io::GimbalMode::IDLE};//初始模式

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();//程序开始运行时间为t0
    uint16_t last_bullet_count = 0;//初始化

    while (!quit) {
      if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
        auto target = target_queue.front();//主线程在检测到目标后，会把目标放进这个队列。
        auto gs = gimbal.state();//云台状态——包括各种角之类的。
        auto plan = planner.plan(target, gs.bullet_speed);//根据目标和云台状态，计算控制计划

        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
          plan.pitch_acc);//上位机向云台发送数据，依次为：控制使能、开火使能、yaw 位置、yaw 速度、yaw加速度、pitch位置、pitch速度、pitch 加速度

             std::this_thread::sleep_for(10ms);//每10毫秒循环一次 原来的
          // std::this_thread::sleep_for(20ms);//每20毫秒循环一次（50Hz），避免串口缓冲区积压
      } else
        std::this_thread::sleep_for(200ms);
    }
  });

  while (!exiter.exit()) {
    mode = gimbal.mode();//读取当前模式

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();//dvice change mode
    }

    camera.read(img, t);
    auto q = gimbal.q(t);//读取四元数
    auto gs = gimbal.state();//云台状态
    recorder.record(img, q, t);//录制视频和IMU数据！这里没有被注释掉！
    solver.set_R_gimbal2world(q);//解算四元数

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto armors = yolo.detect(img);//yolo识别装甲板
      auto targets = tracker.track(armors, t);//跟踪目标，并更新目标状态
      if (!targets.empty())
        target_queue.push(targets.front());//把第一个目标放进队列
      else
        target_queue.push(std::nullopt);//没有目标就放空
    }

    /// 打符——略过
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      auto_aim::Plan buff_plan;
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      }
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);

    } else
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}