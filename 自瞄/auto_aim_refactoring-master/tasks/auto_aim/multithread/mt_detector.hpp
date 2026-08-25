#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tuple>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{
/*多线程装甲板检测，
就是把“装甲板识别/检测”这件事放到独立的线程里跑，
让它和主线程（取图、控制云台/开火、通信等）并行执行。
*/
class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);//读取配置文件，debug模式默认是false（0）

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);//push图像和对应的时间戳，就是相机采图

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();//检测到的装甲板结果列表，暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();//比前者多了一个Mat，很可能是有信息的
  //包含：图像，装甲板列表，时间戳
private:
  ov::Core core_;//openvino核心
  ov::CompiledModel compiled_model_;//编译后的模型
  std::string device_;//用来运行的设备（如：CPU  GPU等）
  YOLO yolo_;//yolo模型（通常为YOLOv5）

  tools::ThreadSafeQueue<
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest>>
    queue_{16, [] { tools::logger()->debug("[MultiThreadDetector] queue is full!"); }};
};//上面tuple里面包含图像 时间戳 推理后图像
  //上面是线程安全队列：一个线程 push，另一个线程 pop，不会数据竞争

}  // namespace multithread
//tuple 是：不可变的有序容器,可以装不同类型的参数
}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP

/*这个类的实质是把“装甲板检测（YOLO 推理）”封装成一个组件：
你不断 push 图像进去，它在后台用 OpenVINO 做推理，
把结果（装甲板列表）通过 pop / debug_pop 取出来*/