#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"
/*Detector / Tracker
        ↓
      Target
        ↓
      Aimer   ←（你这份代码）
        ↓
     Command
        ↓
     Gimbal / CBoard*/
namespace auto_aim
{

struct AimPoint
{
  bool valid;//是否有效——瞄准点能不能用
  Eigen::Vector4d xyza;//目标位置，a一般是装甲板朝向角
};
//Aimer属于决策层
class Aimer
{
public:
  AimPoint debug_aim_point;//调试用
  explicit Aimer(const std::string & config_path);//加载配置文件
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);
//有2个射击模式，可以认为单发/连发，手动/自动
private:
  double yaw_offset_;//云台yaw偏置
  std::optional<double> left_yaw_offset_, right_yaw_offset_;//左右yaw偏移
  double pitch_offset_;//云台pitch偏置
  double comming_angle_;//进入角度
  double leaving_angle_;//离开角度。两者用于装甲板即将转到正面 或转走 时候 选择“最容易打中的装甲“
  double lock_id_ = -1;//锁定的目标ID -1表示未锁定，用来防止频繁切目标
  double high_speed_delay_time_;//高速 → 用更大的预测延迟
  double low_speed_delay_time_;//低速 → 更小延迟
  double decision_speed_;//决策速度
/*目标快 → 预测得更远 目标慢 → 预测得更近
这是为了解决：
高速旋转 / 飞行 → 不预测必空
低速目标 → 过度预测反而打偏*/
  AimPoint choose_aim_point(const Target & target);//核心算法之一：从一个 Target中选择最佳瞄准点
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP