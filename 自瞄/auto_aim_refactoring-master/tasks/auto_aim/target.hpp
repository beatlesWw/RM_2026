#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"
//设置命名空间
namespace auto_aim
{

class Target
{
public:
  ArmorName name;//装甲板名称
  ArmorType armor_type;//装甲板类型
  ArmorPriority priority;//装甲板优先级
  bool jumped;//是否跳步
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);//初始化
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;//获取EKF状态
  const tools::ExtendedKalmanFilter & ekf() const;//获取EKF
  std::vector<Eigen::Vector4d> armor_xyza_list() const;//获取装甲板位置

  bool diverged() const;//是否发散

  bool convergened();//是否收敛

  bool isinit = false;//是否初始化

  bool checkinit();//初始化检查

private:
  int armor_num_;//装甲板数字
  int switch_count_;//切换次数
  int update_count_;//更新次数

  bool is_switch_, is_converged_;//是否切换，是否收敛

  tools::ExtendedKalmanFilter ekf_;//扩展卡尔曼滤波
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
//“我预测的状态 x → 观测到的装甲板位置/测量值”
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;//把状态 x 映射成“我应该看到的装甲板位置”
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;//计算这个映射的导数（EKF 需要）
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP