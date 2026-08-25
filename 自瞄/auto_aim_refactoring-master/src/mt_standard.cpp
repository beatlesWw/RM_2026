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

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";
/*
standard = 标准，表明这是一套“标准/基础的主程序
也就是：
负责 初始化所有模块、主循环逻辑、切换模式、调用自瞄 / 打符 [我们不需要打符]等任务
它相当于你项目里 “默认的运行模式” 的主程序文件。*/
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

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter);

  std::atomic<io::Mode> mode{io::Mode::idle};
  auto last_mode{io::Mode::idle};

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
//打开相机的线程
    while (!exiter.exit()) {
      if (mode.load() == io::Mode::auto_aim) {
        camera.read(img, t);
        detector.push(img, t);
      } else
        continue;
    }
  });

  while (!exiter.exit()) {
    mode = cboard.mode;//读取当前模式

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode.load();
    }

    /// 自瞄
    if (mode.load() == io::Mode::auto_aim) {
      auto [img, armors, t] = detector.debug_pop();
      Eigen::Quaterniond q = cboard.imu_at(t - 1ms);//读取四元数

      // recorder.record(img, q, t);//原本被注释掉的，录制视频和IMU数据

      solver.set_R_gimbal2world(q);//解算四元数

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);//计算云台当前姿态（欧拉角）

      auto targets = tracker.track(armors, t);//跟踪目标，并更新目标状态

      commandgener.push(targets, t, cboard.bullet_speed, ypr);  // 发送给决策线程

    }

    /// 打符
    else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) {
      cv::Mat img;
      Eigen::Quaterniond q;
      std::chrono::steady_clock::time_point t;

      camera.read(img, t);
      q = cboard.imu_at(t - 1ms);

      // recorder.record(img, q, t);

      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      io::Command buff_command;
      if (mode.load() == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      } else if (mode.load() == io::Mode::big_buff) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      cboard.send(buff_command);

    } else
      continue;
  }

  detect_thread.join();

  return 0;
}