#include "math_tools.hpp"

#include <cmath>
#include <opencv2/core.hpp>  // CV_PI

namespace tools
{
double limit_rad(double angle)
{
  while (angle > CV_PI) angle -= 2 * CV_PI;
  while (angle <= -CV_PI) angle += 2 * CV_PI;//使得数值回到[-pi, pi]闭区间范围内
  return angle;
}
//axis0, axis1, axis2: x=0, y=1, z=2(右手系，即zyx)
//extrinsic（英文：外在的）: true为外旋，false为内旋
Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
  if (!extrinsic) std::swap(axis0, axis2);//如果不是外旋，把axis0（z）和axis2（x）交换，是为了把 内旋 转成 等价的外旋 去计算。（hpp文件写false是因为false默认的，可以忽略）
//但是zyx的顺序不能调换！！否则有问题！！！
  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);
  /*4.1 两类欧拉角（你必须知道）
① Tait–Bryan angles（你们在用）
特点：三个轴 两两不同
例子：zyx（yaw–pitch–roll）xyz
数学特征：i ≠ k
② Proper Euler angles（航空 / 机器人老体系）
特点：第 1 次 和 第 3 次 绕同一轴
例子：zxz yxy   数学特征：i == k*/
  if (is_proper) k = 3 - i - j;//只再i==k的情况下运行
  auto sign = (i - j) * (j - k) * (k - i) / 2;//判断旋转轴顺序的“正负方向”，sign只有1和-1两个值
//四元数转欧拉角具体计算
  double a, b, c, d;
  Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }
/*eulers[0] → 第一次旋转角
eulers[1] → 中间旋转角（第二个）
eulers[2] → 最后一次旋转角
*/
  Eigen::Vector3d eulers;
  auto n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  auto half_sum = std::atan2(b, a);
  auto half_diff = std::atan2(-d, c);
//下面四行代码是奇异点（万向节锁）检测
/*β ≈ 0（safe1）或β ≈ π(safe2)时,第一轴和第三轴重合α 和 γ 无法唯一确定*/
  auto eps = 1e-7;
  auto safe1 = std::abs(eulers[1]) >= eps;
  auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
  auto safe = safe1 && safe2;
  if (safe) {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;//正常情况的计算
  } else {
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
    /*干脆承认“不唯一”，然后根据内旋/外旋的语义，固定一个角为 0，
把所有旋转信息塞进另一个角里，保证数值稳定、连续、可用。*/
  }

  for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);//固定欧拉角在一个地方
//i不等于k时候， RM 自瞄用的 ZYX，一定是 !is_proper
  if (!is_proper) {
    eulers[2] *= sign;//保证右手系方向一致
    eulers[1] -= CV_PI / 2;//没有这行，会整体偏90度
  }

  if (!extrinsic) std::swap(eulers[0], eulers[2]);//内旋语义的最终恢复，换回原来的。

  return eulers;
}
//得到了欧拉角后的计算
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
  Eigen::Quaterniond q(R);
  return eulers(q, axis0, axis1, axis2, extrinsic);
}
/*ZYX 顺序的欧拉角（yaw → pitch → roll，内旋）→ 旋转矩阵，矩阵乘法顺序是 z → y → x*/
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)
{
  double roll = ypr[2];
  double pitch = ypr[1];
  double yaw = ypr[0];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };
    //从上到下分别是：新的x轴、y轴、z轴在世界坐标系下的表示
  // clang-format on
  return R;
}
/*把“相机坐标系下的三维点 (x, y, z)”→ 转成“自瞄使用的 yaw / pitch / distance”*/
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

/*计算 xyz2ypd 的雅可比矩阵【把一个函数在某一点附近的变化规律用线性近似表达出来。】用于EKF计算*/
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];

  auto dyaw_dx = -y / (x * x + y * y);
  auto dyaw_dy = x / (x * x + y * y);
  auto dyaw_dz = 0.0;

  auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

  auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

  // clang-format off
  Eigen::MatrixXd J{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };
  // clang-format on

  return J;//得到雅克比矩阵
}
//球坐标的反推与计算雅克比矩阵
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  auto x = distance * std::cos(pitch) * std::cos(yaw);
  auto y = distance * std::cos(pitch) * std::sin(yaw);
  auto z = distance * std::sin(pitch);
  return {x, y, z};
}

Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  double cos_pitch = std::cos(pitch);
  double sin_pitch = std::sin(pitch);

  auto dx_dyaw = distance * cos_pitch * -sin_yaw;
  auto dy_dyaw = distance * cos_pitch * cos_yaw;
  auto dz_dyaw = 0.0;

  auto dx_dpitch = distance * -sin_pitch * cos_yaw;
  auto dy_dpitch = distance * -sin_pitch * sin_yaw;
  auto dz_dpitch = distance * cos_pitch;

  auto dx_ddistance = cos_pitch * cos_yaw;
  auto dy_ddistance = cos_pitch * sin_yaw;
  auto dz_ddistance = sin_pitch;

  // clang-format off
  Eigen::MatrixXd J{
    {dx_dyaw, dx_dpitch, dx_ddistance},
    {dy_dyaw, dy_dpitch, dy_ddistance},
    {dz_dyaw, dz_dpitch, dz_ddistance}
  };
  // clang-format on

  return J;//得到雅克比矩阵
}
//时间差
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)
{
  std::chrono::duration<double> c = a - b;
  return c.count();
}
//得到绝对角【0——pi】
/*当前炮管朝向方向和目标方向或许有偏差，你想知道偏差有多大（不管左偏还是右偏），就可以用这个函数算“夹角大小”。*/
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)
{
  if (vec1.norm() == 0. || vec2.norm() == 0.) {
    return 0.;
  }
  return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

//限制最小最大值，在正负180度之间
double limit_min_max(double input, double min, double max)
{
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
}  // namespace tools