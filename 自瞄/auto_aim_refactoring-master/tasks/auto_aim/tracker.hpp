#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"
/*Tracker = 在连续时间上，决定“我现在在打谁，以及还能不能继续打它”。*/
namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);//读取路径config_path，初始化Solver

  std::string state() const;//查询当前状态
  //传统路径
  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);//跟踪当前帧检测到的装甲板，使用敌方颜色过滤（防止我方颜色错误识别，减少变量），并返回目标列表
  //基于YOLO 识别的全感知融合
  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);//跟踪检测结果，返回目标列表

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;//当前追踪目标
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;//这边有的参数是配置读取的，有的则是动态变化的。

  void state_machine(bool found);//状态机——寻找

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);//设置/确定目标

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);//更新or追踪目标
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP