#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
//MT：Multi-Thread（多线程）
//while (!exiter.exit()) {}不止一个，就说明是多线程
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

using namespace std::chrono;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder(100);  //根据实际帧率调整

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
//读取相机的
    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    auto t0 = std::chrono::steady_clock::now();//程序开始时间就是t0
    /// 自瞄核心逻辑
    auto [img, armors, t] = detector.debug_pop();
    /*它从检测线程拿到：img：当前帧图像  armors：检测到的装甲板列表t：该帧时间戳
   你可以理解为：检测线程在不停地检测，把结果放进队列，主线程从队列里取出来处理。*/
    Eigen::Quaterniond q = cboard.imu_at(t - 1ms);//读取当前云台四元数
    mode = cboard.mode;//读取模式

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;//读取模式
    }

    solver.set_R_gimbal2world(q);//设置云台到世界的旋转矩阵

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0); //计算欧拉角

    auto targets = tracker.track(armors, t);//进行跟踪

    commandgener.push(targets, t, cboard.bullet_speed, ypr);  // 发送给决策线程

    /// debug
    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});
    //还是在这个地方用白字写云台状态
    //下面均为发送到plot里面的参数
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left——依旧优先检验左边的装甲板
      solver.solve(armor);//对左边的装甲板进行解算，转换成target
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();//对target进行追踪与检测

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();//获取装甲板位置
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);//将装甲板的3D点投影到图像上
        tools::draw_points(img, image_points, {0, 255, 0});//绘制装甲板的角点
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;//获取瞄准点
      Eigen::Vector4d aim_xyza = aim_point.xyza;//算出瞄准点的xyz和角度
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);//重投影到图像平面
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});
    /*如果瞄准点有效（valid == true），就用 红色 画点（BGR 中 {0,0,255} 是红色）
     如果无效，就用 蓝色 画点（BGR 中 {255,0,0} 是蓝色）*/
      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    // 云台响应情况
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = ypr[1] * 57.3;
    data["bullet_speed"] = cboard.bullet_speed;

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);//reprojection意思是重投影
    auto key = cv::waitKey(1);
    if (key == 'q') break;//q键退出
  }

  detect_thread.join();//ctrl+c后，等待检测线程结束

  return 0;
}