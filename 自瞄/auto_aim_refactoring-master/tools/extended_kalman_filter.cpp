#include "extended_kalman_filter.hpp"

#include <numeric>
/*概念明晰：x: 状态向量（如位置、速度等）
P: 协方差矩阵（表示状态估计的不确定性）
I: 单位矩阵（用于计算）
x_add: 自定义状态加法函数（处理角度等特殊情况）*/
namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;//初始化——运行过程中：会被更新为实际计算的统计值
}
//是一个函数重载的实现，使用了委托模式（delegation pattern）。
/*这是线性预测的简化版本
参数：只需要 F（状态转移矩阵）和 Q（过程噪声协方差）
适用场景：系统是线性的，状态转移就是简单的矩阵乘法 F * x     */
Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;//协方差矩阵的预测——P协方差矩阵，F状态转移矩阵，Q过程噪声协方差【对系统模型预测不准确的程度】
  x = f(x);//x是当前状态，f是传递近来的状态转移函数
  return x;//返回预测后的状态（也就是预测结果）。
}   //通用版本


//以下是更新
/*z：传感器测量值/观测值
H：观测矩阵（把状态 x 映射到观测空间）
R：观测噪声协方差（测量有多不准）
z_subtract：测量残差的“减法”，默认是 a - b，但可以自定义（比如角度要做 wrap）*/
Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);//H * x是观测函数h，即把状态x映射成“预测观测值”
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;//更新前 （预测后）
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();//计算卡尔曼增益K
  //H * P * H.transpose() + R是协方差矩阵    transpose表示转置矩阵，行列互换
  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();//更新协方差P

  x = x_add(x, K * z_subtract(z, h(x)));//更新状态向量x

  /// 卡方检验
  Eigen::VectorXd residual = z_subtract(z, h(x));//残差
  // 新增检验
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;//用来做卡方检验的统计量
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);//一致性检验统计量

  // 卡方检验阈值（自由度=4，取置信水平95%）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);//失败是1,成功是0，输入队列中

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();//如果当前队列长度大于100,就把最老的元素从队列头部删掉。只保留100个
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);//最近窗口里失败次数
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();///失败率

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;//存入data字典里面，提供画图

  return x;//把更新后的状态向量作为函数结果返回
}

}  // namespace tools