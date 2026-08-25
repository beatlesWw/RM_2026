
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/core/eigen.hpp>

#include "io/camera.hpp"
// #include "io/cboard.hpp"   原来的，目前不要了
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
using namespace std::chrono_literals;  // 添加这行2025.11.7日添加的

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/handeye.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

// 世界坐标到像素坐标的转换

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  auto yaml = YAML::LoadFile(config_path);
  auto height = yaml["height"].as<double>();
  auto grid_num = yaml["grid_num"].as<int>();
  auto grid_size = yaml["grid_size"].as<double>();
  auto delay = yaml["delay"].as<int>();
  //以下四行是新加的【2.1】
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_camera2gimbal(R_camera2gimbal_data.data());
  Eigen::Vector3d t_camera2gimbal(t_camera2gimbal_data.data());
  //  io::CBoard cboard(config_path);原来的
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  std::vector<cv::Point3f> points;
  for (int x = 0; x < grid_num; x++) {
    for (int y = 0; y < grid_num; y++) {
      points.emplace_back(x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
      points.emplace_back(-x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
    }
  }
  //以下5行是新加的。
  Eigen::Vector3d mean_world = Eigen::Vector3d::Zero();
  for (const auto & p : points) {
    mean_world += Eigen::Vector3d(p.x, p.y, p.z);
  }
  mean_world /= static_cast<double>(points.size());
  while (!exiter.exit()) {
    camera.read(img, t);
    //    q = cboard.imu_at(t - 1ms * delay);原来的
    q = gimbal.q(t - 1ms * delay);
    solver.set_R_gimbal2world(q);
    cv::Mat result = img.clone();
    std::vector<cv::Point2f> projectedPoints = solver.world2pixel(points);
    for (const auto & point : projectedPoints) {
      if (point.x >= 0 && point.x < result.cols && point.y >= 0 && point.y < result.rows) {
        cv::circle(result, point, 3, cv::Scalar(255, 255, 255), -1);
      }
    }
    Eigen::Vector3d euler = solver.R_gimbal2world().eulerAngles(2, 1, 0) * 180.0 / M_PI;
    tools::draw_text(result, fmt::format("yaw   {:.2f}", euler[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(result, fmt::format("pitch {:.2f}", euler[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(result, fmt::format("roll  {:.2f}", euler[2]), {40, 120}, {0, 0, 255});
    //到111行都是2.1新加的
    Eigen::Matrix3d R_gimbal2world = solver.R_gimbal2world();
    Eigen::Vector3d cam_in_world = R_gimbal2world * t_camera2gimbal;

    Eigen::Matrix3d R_world2camera = R_camera2gimbal.transpose() * R_gimbal2world.transpose();
    Eigen::Vector3d t_world2camera = -R_camera2gimbal.transpose() * t_camera2gimbal;
    Eigen::Vector3d mean_in_camera = R_world2camera * mean_world + t_world2camera;

    tools::draw_text(
      result,
      fmt::format(
        "cam_w  [{:.2f} {:.2f} {:.2f}]", cam_in_world.x(), cam_in_world.y(), cam_in_world.z()),
      {40, 160}, {0, 255, 0});
    tools::draw_text(
      result,
      fmt::format(
        "pt_w   [{:.2f} {:.2f} {:.2f}]", mean_world.x(), mean_world.y(), mean_world.z()),
      {40, 200}, {0, 255, 0});
    tools::draw_text(
      result,
      fmt::format(
        "pt_c   [{:.2f} {:.2f} {:.2f}]", mean_in_camera.x(), mean_in_camera.y(),
        mean_in_camera.z()),
      {40, 240}, {0, 255, 0});
    tools::draw_text(
      result, fmt::format("front {}", mean_in_camera.z() > 0.0 ? 1 : 0), {40, 280}, {0, 255, 0});
    if (!display) continue;
    cv::imshow("result", result);
    if (cv::waitKey(1) == 'q') break;
  }
}