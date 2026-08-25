#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
/*track行动：从一堆 Armor 中选一个“可信目标”
用 EKF 持续预测 + 更新这个目标
在丢失 / 切换 / 发散时，果断重来*/
namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}//初始化
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}//读取参数

std::string Tracker::state() const { return state_; }//返回当前状态
//下列的追踪是单传感器版本
std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);//时间一致性检查
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";//防止相机掉帧 / 停顿导致 EKF 发散
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板——无人机不能过滤
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.outpost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.outpost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)所以armor.front()是最应该打的目标
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;//表示有没有得到/更新了目标状态
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");//是否偏离
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}
//下面的跟踪是“主相机 + 全向感知”的协同跟踪版本，允许更高优先级目标插队
std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};//创建一个空的装甲板列表，用于临时接收
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};//创建一个空的装甲板列表，用于切换请求。“有一个来自全向感知的、更高优先级目标，建议你为切换做准备”
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }//从队列中取出最新的一帧

  auto dt = tools::delta_time(t, last_timestamp_);//计算时间间隔
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线（0,1为阈值）
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";//保险起见
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;//防止误检
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";//在参数中改min_detect_count_【无人机是5】！！！
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }//要连续看到五帧，才能算检测到
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";//短暂丢失，但是不能够置之不理
  }
//switching表示切换目标状态
  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";//或许存在目标没进入视野就消失了，但是200帧都没看到，就认为丢失
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";//临时丢失如果在一定时期内还没有再度看见，被视为丢失
    }
  }
}
//下面表示“我决定锁谁，并且如何初始化这个目标的物理/滤波模型”
bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;//如果没有就返回

  auto & armor = armors.front();//默认第一个是最重要的
  solver_.solve(armor);///解算的那个装甲板的参数

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);//是否为平衡步兵
  //P0_dig：P₀ 的对角线元素（vector 形式）
  /*翻译
  | 位置 | 物理量    |数值  | 人话解释        |
  | -- | --------- | --- | -----------     |
  | 0  | x         | 1   | 位置初始比较准    |
  | 1  | vx        | 64  | 速度很不确定      |
  | 2  | y         | 1   | 位置比较准        |
  | 3  | vy        | 64  | 速度不确定        |
  | 4  | z         | 1   | 深度较准          |
  | 5  | vz        | 64  | z 方向速度不确定   |
  | 6  | yaw       | 0.4 | 朝向有点不确定     |
  | 7  | yaw_rate  | 100 | 转速极其不确定     |
  | 8  | r         | 1   | 半径一般          |
  | 9  | r_dot     | 1   | 半径变化          |
  | 10 | bias / 其他 | 1   | 其他误差        |*/
  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}
//下面是用当前帧的装甲板观测，去“纠正”上一帧已经存在的目标（EKF 跟踪）
bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);//EKF 预测（Prediction）

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }//在这一帧所有装甲板里：只留下 “兵种相同 + 大小相同“的

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (
      armor.name != target_.name || armor.type != target_.armor_type
      //  || armor.center.x != min_x
    )
      continue;//同一帧内，所有匹配装甲板都会用于更新（把注释掉的那行注释去掉，那么表示就只更新最左侧的）

    solver_.solve(armor);

    target_.update(armor);//对每一个装甲板都进行更新
  }

  return true;
}

}  // namespace auto_aim