#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim
{

class Detector
{
public:
  Detector(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count = -1);//返回装甲板列表

  bool detect(Armor & armor, const cv::Mat & bgr_img);//检测是否为装甲板

  friend class YOLOV8;

private:
  Classifier classifier_;

  double threshold_;//二值化阈值
  double max_angle_error_;//最大角度误差
  double min_lightbar_ratio_, max_lightbar_ratio_;//最小最大灯条比例
  double min_lightbar_length_;//最小灯条长度
  double min_armor_ratio_, max_armor_ratio_;//最小最大装甲板比例
  double max_side_ratio_;//最大边长比例
  double min_confidence_;//最小置信度
  double max_rectangular_error_;//最大矩形误差

  bool debug_;//是否调试
  std::string save_path_;//保存路径

  // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  bool check_geometry(const Lightbar & lightbar) const;
  bool check_geometry(const Armor & armor) const;//几何筛选
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;//名字与类型筛选

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;//区域
  ArmorType get_type(const Armor & armor);//装甲板
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;//归一化中心点（用于控制）

  void save(const Armor & armor) const;//save：保存装甲板截图（用于训练/调试）
  void show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;
};//显示结果

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP