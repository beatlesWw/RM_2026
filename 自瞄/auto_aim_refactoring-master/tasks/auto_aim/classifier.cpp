#include "classifier.hpp"

#include <yaml-cpp/yaml.h>
/*Classifier = 装甲板数字分类器（1~5 / 哨兵 / 非装甲）*/
namespace auto_aim
{
Classifier::Classifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);//读取配置文件
  auto model = yaml["classify_model"].as<std::string>();//读取模型路径
  net_ = cv::dnn::readNetFromONNX(model);//读取模型
  auto ovmodel = core_.read_model(model);//读取模型
  compiled_model_ = core_.compile_model(
    ovmodel, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}//LATENCY 低延迟  适合自瞄

void Classifier::classify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;//如果这块装甲板“没有数字图像”，那它就不可能是有效装甲，直接标记为 not_armor，并退出分类函数。
  }

  cv::Mat gray;//定义一个灰度图
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);//将装甲板的数字图像转换为灰度图

  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));//定义一个32x32的灰度图
  auto x_scale = static_cast<double>(32) / gray.cols;//计算x方向的缩放比例
  auto y_scale = static_cast<double>(32) / gray.rows;//计算y方向的缩放比例
  auto scale = std::min(x_scale, y_scale);//取最小的缩放比例
  auto h = static_cast<int>(gray.rows * scale);//计算缩放后的高度
  auto w = static_cast<int>(gray.cols * scale);//计算缩放后的宽度

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;//保险
  }
  auto roi = cv::Rect(0, 0, w, h);//定义一个矩形区域
  cv::resize(gray, input(roi), {w, h});//将灰度图缩放到32x32

  auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());//将32x32的灰度图转换为blob（二进制大型对象）

  net_.setInput(blob);//将原来的模型灰度化，把已经处理好的输入数据送进神经网络
  cv::Mat outputs = net_.forward();//输出这样的图形

  // softmax——把神经网络输出的一串“任意实数分数”，变成 0～1 之间、且总和为 1 的“概率”
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}//是为了置信度

//“同一个数字分类器，用 OpenVINO 来跑一遍，追求更快 / 更稳的推理”，和上面的步骤异曲同工没有多大的不同。
void Classifier::ovclassify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  // Resize image to 32x32
  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }

  auto roi = cv::Rect(0, 0, w, h);//定义一个矩形区域
  cv::resize(gray, input(roi), {w, h});//将灰度图缩放到32x32
  // Normalize the input image to [0, 1] range
  input.convertTo(input, CV_32F, 1.0 / 255.0);//将32x32的灰度图转换为blob（二进制大型对象）

  ov::Tensor input_tensor(ov::element::f32, {1, 1, 32, 32}, input.data);//把 OpenCV 的 Mat 变成 OpenVINO 的输入格式

  ov::InferRequest infer_request = compiled_model_.create_infer_request();//创建一个推理请求
  infer_request.set_input_tensor(input_tensor);//设置输入张量
  infer_request.infer();//进行推理
/*compiled_model_：模型已经被 OpenVINO 编译过了（加速版）
create_infer_request()：创建一次推理请求
set_input_tensor()：把刚才的输入给模型
infer()：真正跑一次前向传播*/
  auto output_tensor = infer_request.get_output_tensor();//输出张量 output_tensor 就是网络的预测结果。
  auto output_shape = output_tensor.get_shape();
  cv::Mat outputs(1, 9, CV_32F, output_tensor.data());

  // Softmax
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

}  // namespace auto_aim