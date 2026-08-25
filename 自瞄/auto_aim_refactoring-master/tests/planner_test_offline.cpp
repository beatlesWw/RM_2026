#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

/*离线测试程序：不接相机、不跑整套识别链路，而是“虚拟一个在运动的目标 Target”，
然后不断调用 auto_aim::Planner 做瞄准与控制规划，把结果用 Plotter 画出来（或输出/可视化），
用于验证规划器的输出是否平滑、是否跟得上目标、速度/加速度是否合理。*/

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 5.0 | Target角速度(rad/s) }"
  "{@config-path   |     | yaml配置文件路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");//target距离，默认3m
  auto w = cli.get<double>("w");//target角速度 默认5.0 rad/s
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;//对以上进行检查
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_aim::Planner planner(config_path);
  auto_aim::Target target(d, w, 0.2, 0.1);

  auto t0 = std::chrono::steady_clock::now();//取得一个起始时间点进行保存
  /*读取当前时刻（一个时间点 time_point），并把这个时间点存到变量 t0 里（*/

  while (!exiter.exit()) {
    target.predict(0.01);

    auto plan = planner.plan(target, 22);//

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);//循环读取当前时间，计算从开始到现在过了多少秒，
    // 存到data["t"]，作为画图的横轴时间

    data["target_yaw"] = plan.target_yaw;
    data["target_pitch"] = plan.target_pitch;

    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;

    data["plan_pitch"] = plan.pitch;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;

    plotter.plot(data);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}