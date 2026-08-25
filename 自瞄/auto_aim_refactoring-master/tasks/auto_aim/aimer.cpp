#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);//读取参数文档——Aimer 的所有行为都靠配置驱动，不必常常改！
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad  均统一改成弧度制
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad 左右发射补偿（可选）
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};//前置检查，没目标 → 不转、不打
  auto target = targets.front();//取得第一个值

  auto ekf = target.ekf();
  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;//根据目标速度决定预测延时，动态延迟模型

  if (bullet_speed < 14) bullet_speed = 23;//子弹速度保底。最低23m/s，保险。为了解决一些异常。但是如果大于14,小于23,则不会有影响

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;//这一帧目标“被观测到”的时间
  //to_now意思是：把目标状态预测到“现在 + 发弹延迟”的时刻，是真实的物理时间，所以不可控。
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;//dt = (现在 - 观测时刻) + 发弹延迟
    future += std::chrono::microseconds(int(dt * 1e6));//把 timestamp 推到 “子弹真正出膛的那一刻”
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);//else分支是可控/离线/理想化系统，通常用于调试下。
  }

  auto aim_point0 = choose_aim_point(target);//选择瞄准点【只根据“预测到出膛时刻的目标状态”选装甲板】
  debug_aim_point = aim_point0;//完全不考虑初始时间
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    //上面一行原本是注释掉了的
    return {false, false, 0, 0};//不符合解就退出。说明：装甲不在可射击角度 小陀螺判定失败 目标状态异常
  }
/*假设目标“不动”，子弹从当前时刻飞向这个位置
解出的trajectory0.pitch与trajectory0.fly_time是第一次飞行时间估计*/
  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};//解决不出来直接放弃，没关系的
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测，每一次迭代具体数值都不一样，必须要这样子！！！
//以下均为迭代。『邢（疑惑：如果十次迭代还没有达到收敛条件怎么办呢？）』
  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置 【即预测目标在“子弹命中时刻”的位置】
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);//需要重新选择瞄准点
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};//如果解算不出来还是直接放掉了
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;/*意思是说：“再多预测一次，飞行时间几乎不变了”，此时可以退出进程*/
    }
    prev_fly_time = current_traj.fly_time;//否则继续
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);//最终的瞄准点
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负（Trajectory里 pitch 向上是正，但云台协议里 pitch 向上是负）
  return {true, false, yaw, pitch};//返回最终值
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;//定义一个变量：yaw补偿
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
    /*我们的枪应该都是中间的枪口。但如果我们的系统是：
    左右两路发射（哪怕枪管在中间）、发射机制左右不对称、希望对左右策略做补偿……需要进行偏差处理
    左右偏置的参照对象是：机器人/云台的坐标系（也就是 yaw 的 0 度）*/
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;//原来的command.yaw是“默认枪口（中间枪）”算出来的 yaw

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();//ekf状态向量
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();//装甲板位置列表
  auto armor_num = armor_xyza_list.size();//装甲板数量
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};//当前只识别到一个装甲板，没必要纠结，直接打它

  // 整车旋转中心的球坐标yaw。装甲板相对于车中心的角度差
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;//装甲板相对于车中心的角度差列表
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);//delta_angle是装甲板相对于车中心的角度差
    delta_angle_list.emplace_back(delta_angle);//把计算出来的 delta_angle 放进 delta_angle_list 这个容器里。
  }

  // 不考虑小陀螺（if逻辑：“正常模式：不是小陀螺时的选择逻辑”，也就是说转的慢）
  if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;//id_list是装甲板的id列表
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);//选择的是偏移角度是60度以内的，如果超过60度，就跳过；反之则加入列表
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");//或许现在的情况下存在id_list为空的情况
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换（如果每帧都选择更近的那个，会出现“来回切换”的情况——例如打击旋转板）
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }
    /*解释
    lock_id_：当前锁定的装甲板编号  第一次出现两个装甲板时：选择偏角更小的（更靠近正前方）并锁定它（lock_id_）
    之后即使两块装甲板都在范围内，也一直打 lock_id_，不会来回切换。*/
    // 只有一个装甲板在可射击范围内时，退出锁定模式。很清楚，我就不加了
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }
//以下考虑小陀螺。小陀螺特点：旋转很快时，一侧装甲板会不断出现，另一侧不断消失。
  double coming_angle, leaving_angle;//考虑“出现/消失”
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;//装甲板出现的阈值
    leaving_angle = 30 / 57.3;//装甲板消失的阈值（这两者都是前哨站出现的情况下）
  } else {
    coming_angle = comming_angle_;//配置文件的
    leaving_angle = leaving_angle_;//配置文件的
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};//逆时针旋转时，右侧装甲板会先出现，左侧装甲板会逐渐进入（逆时针大于0）
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};//顺时针旋转时，左侧装甲板会先出现，右侧装甲板会逐渐进入（顺时针小于0）
  }

  return {false, armor_xyza_list[0]};//没找到目标
}

}  // namespace auto_aim