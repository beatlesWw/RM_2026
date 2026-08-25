#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;//状态向量——存储当前估计的系统状态
  Eigen::MatrixXd P;//协方差矩阵——存储当前估计的系统状态的不确定性

  ExtendedKalmanFilter() = default;//默认构造函数，允许延迟初始化

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  /*x0: 初始状态向量   P0: 初始协方差矩阵
  x_add: 自定义状态加法函数（默认为向量加法，可用于处理角度等特殊情况）*/    
  
  /*重载是 C++ 中的一个核心特性，允许你定义多个同名函数，只要它们的参数不同。*/
  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);
/*线性预测: predict(F, Q) - 使用状态转移矩阵 F  其中 Q 是过程噪声协方差矩阵 */

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);
/*非线性预测: predict(F, Q, f) - 使用状态转移函数 f【F是状态转移矩阵   Q是过程噪声协方差矩阵】*/
  
/*下列也是重载矩阵*/
Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });
/*线性更新: update(z, H, R) - 使用观测值 z  其中 H 是观测矩阵 R 是观测噪声协方差矩阵 */

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });
/*非线性更新: update(z, H, R, h) - 使用观测值 z  其中 H 是观测矩阵 R 是观测噪声协方差矩阵 h是观测函数*/

  std::map<std::string, double> data;  //卡方检验数据
  std::deque<int> recent_nis_failures{0};//最近的 NIS (Normalized Innovation Squared归一化创新平方) 失败记录
  size_t window_size = 100;//滑动窗口大小
  double last_nis;//最后一次NIS值
//以上四行都是卡方校验，用于评估滤波器性能和检测异常观测。

private:
  Eigen::MatrixXd I;//单位矩阵
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;
/*状态加法函数 作用：定义如何将两个状态向量相加*/
  int nees_count_ = 0;
  /*NEES = Normalized Estimation Error Squared（归一化估计误差平方）
  作用：统计有多少次估计误差超出合理范围*/
  int nis_count_ = 0;/*NIS = Normalized Innovation Squared（归一化新息平方）
  作用：统计有多少次观测值与预测值差异过大*/
  int total_count_ = 0;
  /*总计数器
  作用：记录总共执行了多少次更新
  用途：计算失败率 = nees_count_ / total_count_ 或 nis_count_ / total_count_*/
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP