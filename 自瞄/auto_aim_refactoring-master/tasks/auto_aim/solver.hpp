#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"
/*投影（projection）：世界 → 图像【正向】
反投影（back-projection）：图像 → 世界（不唯一）【反向】
重投影（reprojection）：“先反，再正”
假设世界坐标
      ↓ 投影
理论像素
      ↓
和真实像素对比   重投影 = 用“你算出来的世界结果”，再回到图像检查对不对
*/
namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);//读取（explicit 表示：不允许隐式转换）

  Eigen::Matrix3d R_gimbal2world() const;/*返回一个 3x3 的旋转矩阵 R_gimbal2world 表示“云台坐标系 → 世界坐标系”的旋转关系*/

  void set_R_gimbal2world(const Eigen::Quaterniond & q);//设置云台到世界坐标系的旋转（用一个四元数 q 来设置 R_gimbal2world_）

  void solve(Armor & armor) const;/*输入：Armor 对象（装甲板检测结果）输出：在 armor 里写入解算结果（世界坐标、距离、角度等）
  求解装甲板世界坐标*/
//反投影装甲板（世界坐标 → 图像坐标）（世界坐标 → 图像坐标）
  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double outpost_reprojection_error(Armor armor, const double & pitch);//拼写错误，目前已经改正
  //前哨站重投影偏差  
  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);
  //上面是把一组「世界坐标系中的 3D 点」，通过相机模型，投影成「图像上的 2D 像素点」。
private:
  cv::Mat camera_matrix_;//相机内参矩阵
  cv::Mat distort_coeffs_;//相机畸变系数
  Eigen::Matrix3d R_gimbal2imubody_;//云台到IMU坐标系的旋转矩阵
  Eigen::Matrix3d R_camera2gimbal_;//相机到云台坐标系的旋转矩阵
  Eigen::Vector3d t_camera2gimbal_;//相机到云台坐标系的平移向量
  Eigen::Matrix3d R_gimbal2world_;//云台到世界坐标系的旋转矩阵

  void optimize_yaw(Armor & armor) const;//优化yaw角（使它最佳化）——找到 重投影误差最小 的装甲朝向

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;//装甲板重投影偏差
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;//SJTU成本函数——鲁棒性更强
    /*cv_refs	真实检测到的装甲角点  cv_pts	理论重投影的角点
    inclined	装甲倾角（权重调节）*/
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP