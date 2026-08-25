#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>

namespace tools
{
// 将弧度值限制在(-pi, pi]【范围正负180度，把超过这个范围的角都转换成这个内部的角度】
double limit_rad(double angle);

//RM 自瞄里，zyx ≈ yaw → pitch → roll
//在以下：球坐标 → 用的是 ypd（yaw, pitch, distance）
//姿态 / 旋转 → 用的是 ypr（yaw, pitch, roll）
// 四元数转欧拉角（rpy）
// x = 0, y = 1, z = 2（编号：意思是x轴为0,y轴为1,z轴为2）
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转（zyx）：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);
//extrinsic = false 内旋 基本都是内旋
/*Extrinsic（外旋）—— 每次都绕世界坐标系
Intrinsic（内旋）—— 每次绕当前自身坐标系*/
// 旋转矩阵转欧拉角（rpy）
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转（zyx）：axis0=2, axis1=1, axis2=0
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 欧拉角转旋转矩阵：用途——A 坐标系变换 B：世界系 → 相机系 C：枪管方向计算
// zyx:先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转（zyx）
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标系转球坐标系——最重要的一组！！
// ypd为yaw、pitch、distance的缩写
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标系转直角坐标系——反过来推
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// 球坐标系转直角坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间差a - b，单位：s
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

// 向量夹角 总是返回 0 ~ pi 来自SJTU
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

// 返回输入值的平方
template <typename T>
T square(T const & a)
{
  return a * a;
};

double limit_min_max(double input, double min, double max);//使得不超过上下限
}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP