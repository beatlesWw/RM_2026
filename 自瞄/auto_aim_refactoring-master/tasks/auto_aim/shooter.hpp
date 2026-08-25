#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);//读取

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;//上一个指令
  double judge_distance_;//判断是否射击的距离
  double first_tolerance_;//第一次判断的容差，更严格
  double second_tolerance_;//第二次判断的容差，更加宽松
  bool auto_fire_;//是否自动射击
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP