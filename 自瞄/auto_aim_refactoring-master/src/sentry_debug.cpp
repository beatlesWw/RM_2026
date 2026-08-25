#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  io::ROS2 ros2;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::Camera back_camera("configs/camera.yaml");
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);//q 是 IMU 读取的四元数（云台姿态）

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);//计算欧拉角

    auto armors = yolo.detect(img);//用yolo来检测装甲板

    decider.get_invincible_armor(ros2.subscribe_enemy_status());//获取无敌装甲板信息【例如前哨战没打完的工程】

    decider.armor_filter(armors);//装甲板过滤——去掉不可打击的装甲板

    decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());//获取自动瞄准目标（从 ROS2 获取外部目标）

    decider.set_priority(armors);//设置打击的优先级（排序）

    auto targets = tracker.track(armors, timestamp);//把检测结果 armors 转换成 targets：跟踪目标，更新目标状态

    io::Command command{false, false, 0, 0};//初始化命令

    // 全向感知逻辑
    // 当前目标丢失时，不再只依赖“前置相机+装甲板检测”，而是用多路感知（多相机、多算法、ROS2信息等）去重新寻找目标并做决策。
    if (tracker.state() == "lost")
      command = decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera);//当丢失视野的时候，让云台扫描 用多个相机一起找目标（前置/后置/左右） 结合敌方状态信息（ROS2）选择一个新的目标并输出控制命令
    else
      command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);//视野里面有目标：不需要全向感知，只要精确瞄准和开火即可。

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    cboard.send(command);//决定是否开火，并把命令发送给云台/底盘。

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);//获取目标的装甲板和跟踪目标信息

    ros2.publish(target_info);//再通过ros发布

    /// debug
    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});//依旧写追踪状态。

    nlohmann::json data;

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
      }  //always left——日常选择左边的装甲板为最终目标
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();//如果target存在，那么选择第一个target进行debug

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();//获取装甲板位置
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);//将装甲板的3D点投影到图像上（重投影）
        tools::draw_points(img, image_points, {0, 255, 0});//绿色表示跟踪器估计的装甲板位置，就是标识哪个是应该打得装甲板
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

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
    data["gimbal_yaw"] = gimbal_pos[0] * 57.3;
    data["gimbal_pitch"] = -gimbal_pos[1] * 57.3;
    data["shootmode"] = cboard.shoot_mode;
    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["cmd_shoot"] = command.shoot;
    }

    data["bullet_speed"] = cboard.bullet_speed;

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }
  return 0;
}