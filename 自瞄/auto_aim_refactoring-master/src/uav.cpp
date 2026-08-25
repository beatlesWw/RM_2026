#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
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
/*
总体看：
循环：
 ├─ 拍图（Camera）
 ├─ IMU姿态（CBoard）
 ├─ 模式判断（CBoard.mode）
 ├─ 若为自瞄：
 │   ├─ 检测装甲板
 │   ├─ 跟踪目标
 │   ├─ 解算世界坐标
 │   ├─ 预测瞄准角
 │   ├─ 决策开火
 │   └─ 发送控制
 ├─ 若为打符：（在这样的情况下，无人机不打符，所以这段不用看，直接自瞄就行了）
 │   ├─ 检测扇叶
 │   ├─ 解算角度与旋转
 │   ├─ 预测击打时机
 │   └─ 发送控制
 └─ 等待下一帧*/
const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

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
/*Exiter：监听退出信号（例如按 ESC 或 Ctrl+C），让主循环能优雅地退出；
Plotter：可能用于绘制调试图（实时角度、速度、误差曲线）；
Recorder：记录图像、IMU或预测数据到文件，方便离线分析。*/

  /*Camera：封装大恒或其他工业相机的读取逻辑，输出 cv::Mat 图像；
CBoard：通信板（Controller Board）对象，负责：
获取 IMU 姿态；读取当前“模式”；发送射击/瞄准指令到下位机。
这里的 cboard.mode 就是整机工作状态（比如：自瞄、小符、大符、哨兵模式、空闲等）。*/
  io::Camera camera(config_path);//初始化
  io::CBoard cboard(config_path);//初始化

  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  // auto_aim::YOLO yolo(config_path);
  //YOLO这一行原本被注释掉了
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);


  //下面都是打符相关的——忽略
  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);
//上面都是打符相关的——忽略
  cv::Mat img;//opencv里面的
  Eigen::Quaterniond q;//四元数
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;
    // recorder.record(img, q, t);
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    /// 自瞄
    if (mode == io::Mode::auto_aim || mode == io::Mode::outpost) {
      solver.set_R_gimbal2world(q);//解算四元数

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);//计算云台当前姿态（欧拉角）

      auto armors = detector.detect(img);//检测装甲板——最后选择一个装甲板进行跟踪（通常是最左边的）

      auto targets = tracker.track(armors, t);//跟踪选定的目标

      auto command = aimer.aim(targets, t, cboard.bullet_speed);//瞄准计算，决定瞄准哪里

      command.shoot = shooter.shoot(command, aimer, targets, ypr);//决定是否开火

      cboard.send(command);//把命令发给云台
    }

    /// 打符——这个我们无人机并不负责这个，不好弄，打符不用看
    else if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      io::Command buff_command;
      if (mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      } else if (mode == io::Mode::big_buff) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      cboard.send(buff_command);
    }

    else
      continue;
  }

  return 0;
}