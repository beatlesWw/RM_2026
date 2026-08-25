#include "img_tools.hpp"
//根据hpp文件实现具体功能
namespace tools
{ //画一个实心点————大概是准心，具体参数看hpp文件
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  cv::circle(img, point, radius, color, -1);
}
//画一组点————形成多个轮廓
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}
//画一组点（浮点转整型）——思想：高精度接口 → 复用低精度实现
void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  std::vector<cv::Point> int_points(points.begin(), points.end());
  draw_points(img, int_points, color, thickness);
}
//写文字
void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

}  // namespace tools