#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
//MPC = Model Predictive Control（模型预测控制）——就是predict
using namespace std::chrono_literals;//让你能写这种“带单位”的时间字面量：1ms、10s

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";
//读取参数
int main(int argc, char * argv[])
{
  tools::Exiter exiter;//退出程序 具体见exiter工具
  tools::Plotter plotter;//绘图，具体见tools

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);//这里并没有注释掉，用了YOLO ，必然有OpenVINO
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);/*主线程负责检测与跟踪，得到 Target 后放入队列。
规划线程从队列取 Target，并根据它计算控制命令。std::optional 表示可能没有目标（nullopt 表示空）。*/

  std::atomic<bool> quit = false;//用于通知规划线程退出
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();//程序开始运行时间为t0
    uint16_t last_bullet_count = 0;//记录上一次读取的云台的子弹计数，用于判断是否发生了“开火”动作
//在不退出的情况下，持续读取的循环。它负责实时控制云台（yaw/pitch），并把每一帧的状态数据发送给 Plotter 用于画图。
    while (!quit) {
      auto target = target_queue.front();//主线程在检测到目标后，会把目标放进这个队列。
      auto gs = gimbal.state();//云台状态——包括各种角之类的。
      auto plan = planner.plan(target, gs.bullet_speed);//根据目标和云台状态，计算控制计划

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);//依次为：控制使能、开火使能、yaw 位置、yaw 速度、yaw加速度、pitch位置、pitch速度、pitch 加速度

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;//如果开火，子弹计数会增加

      nlohmann::json data;//接下来的数据传入plotter
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_x"] = target->ekf_x()[0];   //x
        data["target_y"] = target->ekf_x()[2];   //y
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
        data["w"] = target->ekf_x()[7];
        
        // 直接使用EKF中已经计算好的距离残差数据
        data["residual_distance"] = target->ekf().data.at("residual_distance");
        data["residual_yaw"] = target->ekf().data.at("residual_yaw");
        data["residual_pitch"] = target->ekf().data.at("residual_pitch");
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

          //  std::this_thread::sleep_for(10ms);//每10毫秒循环一次 原来的
      std::this_thread::sleep_for(10ms);//每20毫秒循环一次（50Hz），避免串口缓冲区积压
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;//时间戳
  // auto last_time = std::chrono::steady_clock::now(); //3.3debug弄完结束
//下面这部分循环负责：读图像 → 检测 → 跟踪 → 发送目标给规划线程 → 显示重投影图像。
  while (!exiter.exit()) {
    // auto loop_start = std::chrono::steady_clock::now();//3.3debug，弄完结束
    camera.read(img, t);
    auto q = gimbal.q(t);//读取当前云台四元数

    solver.set_R_gimbal2world(q);//设置云台到世界坐标系的旋转矩阵，供solver使用
    auto armors = yolo.detect(img);//检测装甲板（不仅一个）
    auto targets = tracker.track(armors, t);//追踪目标，更新目标状态
    //到125都是为了看帧率
    // auto now = std::chrono::steady_clock::now();
    // auto dt = tools::delta_time(now, last_time);
    // last_time = now;
    // tools::logger()->info("Main loop FPS: {:.2f}", 1.0 / dt);
    if (!targets.empty())
      target_queue.push(targets.front());//检测到了目标，把目标放入队列，供规划线程使用
    else
      target_queue.push(std::nullopt);//没有目标就放空

    if (!targets.empty()) {
      auto target = targets.front();//这段处理的是跟踪结果，前面是检测结果——先决定选择哪个目标，这里决定追踪它

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();//装甲板位姿列表
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});//绿色表示跟踪器估计的装甲板位置，就是标识哪个是应该打得装甲板
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;//取瞄准点
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);//解算后的瞄准点投影到图像平面
      tools::draw_points(img, image_points, {0, 0, 255});//绘制准心——即瞄准的点
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;//q键退出
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}