#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;//必须要有开火命令 有目标 且开了自瞄档位，才能够开火！！

  auto target_x = targets.front().ekf_x()[0];//目标相对于云台的x坐标
  auto target_y = targets.front().ekf_x()[2];//目标相对于云台的y坐标
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
   /*如果 距离 > judge_distance_（远）→ 使用 second_tolerance_（更宽松）
   否则（近）→ 使用 first_tolerance_（更严格）
    这很合理：目标越远，瞄准误差越大，所以容差要大一点。*/
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  //判断是否稳定，决定是否开火。
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值（std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance）
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim