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
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";
//@config-path：必选参数（如果没有传，则默认 configs/uav.yaml），差不多在初始化
using namespace std::chrono_literals;
//cli 是 Command Line Interface（命令行界面）的缩写。
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
//以下都是需要读取的参数
  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;//四元数
  std::chrono::steady_clock::time_point t;//时间戳

  auto mode = io::Mode::idle;//初始模式（我们只需要自瞄！！！！）
  auto last_mode = io::Mode::idle;

  auto t0 = std::chrono::steady_clock::now();//程序开始时间就是t0

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) {
    tools::logger()->error("Camera returned empty image!");
    continue;
} else {
    tools::logger()->info("Image size: {}x{}", img.cols, img.rows);
}

    q = cboard.imu_at(t - 1ms);//t - 1ms，目的是时间对齐，用 t - 1ms 取更“接近”相机帧的 IMU 数据
    mode = cboard.mode;//当前模式
    // recorder.record(img, q, t);//录制视频和 IMU 数据，这个原来就是被注释掉的。
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;//模式转换
    }

    /// 自瞄
    solver.set_R_gimbal2world(q);//读取四元数，设置云台到世界坐标系的旋转矩阵，供solver使用

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);//把旋转矩阵转换成 yaw/pitch/roll（云台欧拉角）。

    auto armors = detector.detect(img);//检测装甲板（不仅一个）

    auto targets = tracker.track(armors, t);//追踪目标，更新目标状态
/*数据关联（判断哪一个装甲板是同一个目标），卡尔曼滤波预测与更新，输出当前目标状态（位置、速度、角度等）*/
    auto command = aimer.aim(targets, t, cboard.bullet_speed);//瞄准器计算“要瞄准哪里”。

    command.shoot = shooter.shoot(command, aimer, targets, ypr);//射击控制器shoot决定“要不要开火”。

    cboard.send(command);//最终发给云台

    /// debug——当前帧图像 img 上画一个文字，用来显示 跟踪器的状态。
    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});
//fmt::format("[{}]"是用来格式化字符串的， {10, 30} 是文字位置——左上角为（0,0），{255, 255, 255}颜色为白色。
    nlohmann::json data;//创建一个 JSON 数据对象 data，并把当前时间 t（相对于程序启动时间）存进去。
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);//计算时间差，单位秒（与开始时间相比）
//这样的data不会出现在终端，但是会帮助plotter进行数据绘制。
    // 装甲板原始观测数据
    data["armor_num"] = armors.size();//装甲板数量给plotter
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left——非常重要！！识别一直是左边的装甲板
      solver.solve(armor);//把装甲板数据传给solver进行位姿解算
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;//写进data。1 rad ≈ 57.2957795 degrees
    }

    if (!targets.empty()) {
      auto target = targets.front();//这段处理的是跟踪结果，前面是检测结果——先决定选择哪个目标，这里决定追踪它

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});//绿色表示跟踪器估计的装甲板位置，就是标识哪个是应该打得装甲板
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;//取瞄准点
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);//解算后的瞄准点投影到图像平面
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});//绘制准心。蓝色：有效瞄准点
      else
        tools::draw_points(img, image_points, {255, 0, 0});//红色——无效瞄准点

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
    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["cmd_shoot"] = command.shoot;
    }
    plotter.plot(data);
//data的一堆数据见plotter
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;//退出键是 q。
  }

  return 0;
}