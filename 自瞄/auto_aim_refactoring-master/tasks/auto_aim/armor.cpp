#include "armor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace auto_aim
{
Lightbar::Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id)
: id(id), rotated_rect(rotated_rect)
{
  std::vector<cv::Point2f> corners(4);
  rotated_rect.points(&corners[0]);
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  center = rotated_rect.center;
  top = (corners[0] + corners[1]) / 2;
  bottom = (corners[2] + corners[3]) / 2;
  top2bottom = bottom - top;

  points.emplace_back(top);
  points.emplace_back(bottom);

  width = cv::norm(corners[0] - corners[1]);
  angle = std::atan2(top2bottom.y, top2bottom.x);
  angle_error = std::abs(angle - CV_PI / 2);
  length = cv::norm(top2bottom);
  ratio = length / width;
}

//传统构造函数——只依赖灯条位置、角度、长度、矩形误差等。不依赖数字识别！！
Armor::Armor(const Lightbar & left, const Lightbar & right)
: left(left), right(right), duplicated(false)
{
  color = left.color;
  center = (left.center + right.center) / 2;

  points.emplace_back(left.top);//[0] 左上
  points.emplace_back(right.top);   // [1] 右上  
  points.emplace_back(right.bottom); // [2] 右下
  points.emplace_back(left.bottom);//[3] 左下 四个角点顺时针

  auto left2right = right.center - left.center;//左右灯条的中点连线
  auto width = cv::norm(left2right);//左右灯条的中点连线长度
  auto max_lightbar_length = std::max(left.length, right.length);//左右灯条的长度
  auto min_lightbar_length = std::min(left.length, right.length);//左右灯条的长度
  ratio = width / max_lightbar_length;//长宽比
  side_ratio = max_lightbar_length / min_lightbar_length;//长灯条与短灯条的长度之比（装甲板必须满足“长宽比合适 + 两灯条长度差不大”）

  auto roll = std::atan2(left2right.y, left2right.x);//roll 表示 装甲板的“横向方向”（从左到右）
  auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);//左灯条的方向与roll的差值
  auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);//右灯条的方向与roll的差值
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);//只要有一条灯条不垂直，就说明这个“装甲板”不靠谱
}

//神经网络构造函数
Armor::Armor(
  int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints)
: class_id(class_id), confidence(confidence), box(box), points(armor_keypoints)
//confidence为置信度，box为检测框，
{//疑惑，四个角点顺序是什么？见40-43
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;//四个角点求平均，就是矩形的中心点。
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);//左灯条的宽度
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);//右灯条的宽度
  auto max_width = std::max(left_width, right_width);//左右灯条的宽度
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);//左灯条的长度
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);//右灯条的长度
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;//左右中心连线
  auto roll = std::atan2(left2right.y, left2right.x);//roll 表示 装甲板的“横向方向”（从左到右）
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
  //上面都是举行误差计算，过滤不规则装甲板
  ratio = max_length / max_width;
  // color = class_id == 0 ? Color::blue : Color::red;
  //属性映射
  if (class_id >= 0 && class_id < armor_properties.size()) {
    auto [color, name, type] = armor_properties[class_id];
    this->color = color;
    this->name = name;
    this->type = type;
  } else {
    this->color = blue;      // Default默认蓝色
    this->name = not_armor;  // Default默认不是装甲板
    this->type = small;      // Default默认小装甲板
  }
}

//神经网络ROI构造函数【相较但一的神经网络，加上了偏移值】(神经网络是对裁剪后的 ROI 进行检测的)
Armor::Armor(
  int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
  cv::Point2f offset)
: class_id(class_id), confidence(confidence), box(box), points(armor_keypoints)
{
  std::transform(
    armor_keypoints.begin(), armor_keypoints.end(), armor_keypoints.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });//把 传入的关键点 vector（ROI坐标）转换成 原图坐标。
  std::transform(
    points.begin(), points.end(), points.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });//把 Armor 类成员变量 points（也就是保存的关键点）也转换成原图坐标。
  //以下内容同神经网络的  
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  // color = class_id == 0 ? Color::blue : Color::red;

  if (class_id >= 0 && class_id < armor_properties.size()) {
    auto [color, name, type] = armor_properties[class_id];
    this->color = color;
    this->name = name;
    this->type = type;
  } else {
    this->color = blue;      // Default
    this->name = not_armor;  // Default
    this->type = small;      // Default
  }
}

// YOLOV5构造函数
Armor::Armor(
  int color_id, int num_id, float confidence, const cv::Rect & box,
  std::vector<cv::Point2f> armor_keypoints)
: confidence(confidence), box(box), points(armor_keypoints)
{
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  color = color_id == 0 ? Color::blue : color_id == 1 ? Color::red : Color::extinguish;//蓝0红1,否则灭。
  name = num_id == 0  ? ArmorName::sentry
         : num_id > 5 ? ArmorName(num_id)
                      : ArmorName(num_id - 1);  //TODO 考虑Bb。根据不同数据类别读取不同名字
  type = num_id == 1 ? ArmorType::big : ArmorType::small;
}

// YOLOV5+ROI构造函数（同前面）
Armor::Armor(
  int color_id, int num_id, float confidence, const cv::Rect & box,
  std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)
: confidence(confidence), box(box), points(armor_keypoints)
{
  std::transform(
    armor_keypoints.begin(), armor_keypoints.end(), armor_keypoints.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });
  std::transform(
    points.begin(), points.end(), points.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  color = color_id == 0 ? Color::blue : color_id == 1 ? Color::red : Color::extinguish;
  name = num_id == 0 ? ArmorName::sentry : num_id > 5 ? ArmorName(num_id) : ArmorName(num_id - 1);
  type = num_id == 1 ? ArmorType::big : ArmorType::small;
}

}  // namespace auto_aim