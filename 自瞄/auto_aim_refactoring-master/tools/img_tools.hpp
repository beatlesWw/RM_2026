#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
//作用：声明了一组“图像绘制辅助函数”，用于在 OpenCV 的图像上画点、画文字，是在opencv的基础上进行再次封装。
namespace tools
{//画一个点【红色，半径为3】
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);
//画多个点【红色，粗细为2】——用于装甲板角点、特征点【整数】
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

//画多个点【红色，粗细为2】——对上一个的函数重载【浮点数】
void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

//画文字【黄色，字体大小为1.0，粗细为2】——用途是显示 FPS，显示距离 / yaw / pitch，显示状态信息
void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP