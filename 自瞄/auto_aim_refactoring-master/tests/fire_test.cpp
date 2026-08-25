#include <fmt/format.h>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

/*不做视觉、不做解算，只是按时间周期给云台下发指令，用来验证：
串口 / CAN 是否通
云台是否能收到 mode
发射逻辑是否按预期触发
本质是个 Gimbal I/O 心跳 + fire 测试器。
警告：
如果你在实车上跑这个，一定要确认：
弹仓空    发射器断电 / 模拟模式
否则这是全自动周期发射代码
*/

// 定义命令行参数
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;//录制器
  tools::Exiter exiter;//退出器

  // 初始化云台
  io::Gimbal gimbal(config_path);
  io::VisionToGimbal plan;
  auto last_fire_t = std::chrono::steady_clock::now();//记录上次发射时间
  auto fire_start_t = std::chrono::steady_clock::now();//记录发射信号开始时间
  bool is_firing = false;//是否正在发射
  plan.yaw = 0;
  plan.yaw_vel = 0;
  plan.yaw_acc = 0;
  plan.pitch = 0;
  plan.pitch_vel = 0;
  plan.pitch_acc = 0;//初始化，把包填满

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    auto gs = gimbal.state();
    
    // 每1.6秒开始一个新的发射周期
    if(!is_firing && tools::delta_time(now, last_fire_t) > 1.600) {
        is_firing = true;
        fire_start_t = now;
        last_fire_t = now;
        tools::logger()->debug("fire start!");
    }
    
    // 发射信号持续100ms后结束
    if(is_firing && tools::delta_time(now, fire_start_t) > 0.100) {
        is_firing = false;
        tools::logger()->debug("fire end!");
    }
    
    plan.mode = is_firing ? 2 : 1;


    gimbal.send(plan);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    if (plan.mode != 0) {
      data["shoot"] = plan.mode == 2 ? 1 : 0;
    }

    plotter.plot(data);//调试可视化

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
