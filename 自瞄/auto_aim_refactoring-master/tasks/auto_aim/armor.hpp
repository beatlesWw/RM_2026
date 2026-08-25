#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
//Armor（装甲板）相关的头文件，主要定义了 装甲板与灯条的结构，
namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,//灯灭了
  purple   //紫色
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};
//装甲板类型——分为大、小2个
enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};
//装甲板名称
enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};
/*sentry：哨兵 outpost：前哨（你问的）base：基地 not_armor：并非装甲板（误检)*/
//装甲板优先级
enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off 
// 下列为属性表
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on
/*例如：{blue, sentry, small} 表示：蓝色哨兵小装甲板{red, base, big} 表示：红色基地大装甲板*/
//灯条结构体（我终于知道什么是结构体了，原来我学过。。。。）
struct Lightbar
{
  std::size_t id;//灯条编号
  Color color;
  cv::Point2f center, top, bottom, top2bottom;//灯条中心点、上端点、下端点、从 top 到 bottom的向量（方向向量）
  std::vector<cv::Point2f> points;//灯条的四个顶点
  double angle, angle_error, length, width, ratio;//灯条的角度、角度误差【表示灯条与“理想竖直”或“预期角度”的偏差用于过滤不符合规则的灯条（例如太斜的灯条）】、长度、宽度、长宽比【用于判断灯条是否过粗或过细（过滤干扰）】
  cv::RotatedRect rotated_rect;//OpenCV 的旋转矩形结构体

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);//当你检测到一个灯条的旋转矩形时，会用这个构造函数创建灯条对象。
  Lightbar() {};//默认构造函数（空构造）
};

struct Armor
{
  Color color;
  Lightbar left, right;     //used to be const  左右灯条
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;//装甲板的四个顶点

  double ratio;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio;         // 长灯条与短灯条的长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2的差值
/*以上三个都是用来过滤装甲板，提高准确性*/
  ArmorType type;
  ArmorName name;
  ArmorPriority priority;
  int class_id;
  cv::Rect box;
  cv::Mat pattern;
  double confidence;
  bool duplicated;//复制

  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d xyz_in_world;   // 单位：m
  Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
  Eigen::Vector3d ypr_in_world;   // 单位：rad
  Eigen::Vector3d ypd_in_world;   // 球坐标系
  /*xyz：三维坐标（m)
  ypr：yaw pitch roll（弧度）
  ypd：球坐标（yaw pitch distance）*/
  double yaw_raw;  // rad

  Armor(const Lightbar & left, const Lightbar & right);//用左右灯条配备装甲板（传统的）
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);//（神经网络的）用装甲板的类别ID、置信度、边界框和关键点创建装甲板
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);//神经网络ROI构造函数——用装甲板的类别ID、置信度、边界框、关键点和偏移量（比上面多了个这个）创建装甲板
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);//YOLOV5构造函数——用装甲板的颜色数字ID、置信度、边界框和关键点创建装甲板
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);//YOLOV5+ROI构造函数——用装甲板的颜色数字ID、置信度、边界框、关键点和偏移量创建装甲板
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP