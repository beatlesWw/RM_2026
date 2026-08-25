#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();//读取路径

  auto model = core_.read_model(model_path);//从 model_path 指向的模型文件里把网络读出来
  ov::preprocess::PrePostProcessor ppp(model);//创建一个“预处理/后处理配置器”，配置模型的输入输出处理规则。
  auto & input = ppp.input();//拿到 上面 [ppp] 里“输入端”的配置接口引用（

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO——风险标记，这边没有弄好
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);
   /*  input.tensor()表示配置喂进去的原始数据
    set_element_type(ov::element::u8)表示输入数据类型为uint8_t
    set_shape({1, 640, 640, 3})表示输入数据的形状为[1, 640, 640, 3]
    set_layout("NHWC")表示输入数据的布局为NHWC——N：一次推理的图片（1次1张），H：高度，W：宽度，C：通道数,和上面对应！
    set_color_format(ov::preprocess::ColorFormat::BGR)表示输入数据的颜色格式为BGR【OpenCV里面就是这样子】*/ 
  input.model().set_layout("NCHW");
  /*声明外部输入是NHWC  期望模型是NCHW  OpenVINO负责做layout转换*/
  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    //上一行原本就是被注释掉了的，表示OpenVINO默认使用线性插值，不会把你的图像缩放到640*640
    .scale(255.0);
   /*input.preprocess()配置输入数据的预处理步骤
    convert_element_type(ov::element::f32)表示将输入数据类型从u8转换为float32[浮点数]
    convert_color(ov::preprocess::ColorFormat::RGB)表示将输入数据的颜色格式从BGR转换为RGB
    scale(255.0)表示将输入数据的值除以255.0，进行归一化操作*/
  model = ppp.build();//按照上面的进行构建
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));
   //THROUGHPUT（吞吐优先）：更偏向整体处理量/并行能力，通常适合想提高整体帧率。
   //（对比）LATENCY（延迟优先）：更偏向单次推理尽量快。
  tools::logger()->info("[MultiThreadDetector] initialized !");
}
//640*640：YOLOv5和v11都是，但是YOLOv8是416*416
/*把原始图 img 等比例缩放（不拉伸）到“能放进 640×640 的网络输入框里”所需要的缩放比例 scale，
并得到缩放后的新尺寸 h、w*/
void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));//先创建一张全黑的 640×640 图（黑底）
  auto roi = cv::Rect(0, 0, w, h);//定义一个矩形区域roi
  cv::resize(img, input(roi), {w, h});//将img等比例缩放到w×h，贴到input(ROI)的左上角,左上角仍然是缩放后的图像，右下剩余区域仍然黑色。
  /*做一次 OpenVINO 推理任务的“创建→喂数据→异步启动→丢进队列”*/
  auto input_port = compiled_model_.input();//拿到这个模型的输入端口描述（输入节点信息）。
  auto infer_request = compiled_model_.create_infer_request();//创建一个推理请求对象
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);//用 OpenVINO 的 ov::Tensor 包装你前面做好的 input 图像数据。
  /*这里是零拷贝引用（tensor 直接用 input 的内存），所以 推理没结束之前这块内存要保持有效。*/
  infer_request.set_input_tensor(input_tensor);//将输入数据设置到推理请求中
  infer_request.start_async();//异步启动推理【异步：返回时推理可能还没算完，线程不会被卡住。】

  queue_.push({img.clone(), t, std::move(infer_request)});//作用：把这次任务打包塞进线程安全队列 queue_，队列里放的是一个 tuple
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [img, t, infer_request] = queue_.pop();//quene_进程里面存储的是tuple元素（原图 时间戳 推理请求对象）
  infer_request.wait();//用的是异步推理，所以需要wait()等待推理才真正完成。等完后，输出tensor才是有效的

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();//推理请求对象的网络输出（比如 YOLO 的检测头输出）。
  auto output_shape = output_tensor.get_shape();//output_shape是输出张量的形状（维度信息）【张量是一种映射】
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  /*这行没有拷贝数据，是“包装/视图”：
把 output_tensor.data()（float 指针）当成一个 OpenCV 矩阵来用
CV_32F：说明输出是 float 数据
output_shape[1]、output_shape[2]：把输出当成二维矩阵来解释
（很多 YOLO 输出常见类似 [1, rows, cols] 的形状，因此用 [1] [2]）*/
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);//重新 or 重复计算scale ，算出当时letterbox里面缩放比例的scale
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI——没有处理“先裁剪 ROI 再检测”带来的坐标偏移/缩放差异

  return {std::move(armors), t};//返回装甲板检验结果列表，与对应的时间戳
}

//下面几乎和上面的一模一样，但是debug会把原图一起返回，方便你做调试显示
std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {img, std::move(armors), t};//比上面多了返回原图
}

}  // namespace multithread

}  // namespace auto_aim
