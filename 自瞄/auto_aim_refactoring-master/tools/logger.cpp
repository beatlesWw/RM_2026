#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
// #include <string>
/*代码实现：第一次调用 tools::logger() 时初始化 logger（包括文件和控制台输出）
之后每次调用都返回同一个 logger 实例。*/
namespace tools {
std::shared_ptr<spdlog::logger> logger_ = nullptr;//空指针常量，不指向任何对象。
//void set_logger()对这个进行初始化
void set_logger() {
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log",
                               std::chrono::system_clock::now());// 生成日志文件名（按时间）。实例路经：logs/2026-01-19_10-23-45.log
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::debug);
/*basic_file_sink_mt：多线程安全的文件 sink
true：表示覆盖写入（每次运行覆盖同名文件）。false：追加写入（保留原文件内容，继续写在后面）
设置级别为 debug（意味着 debug 及以上都会写文件*/
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);
/*控制台输出（带颜色）——颜色是自动带的
同样记录 debug 及以上日志
debug 级别最详细
级别从低到高：
trace < debug < info < warn < error < critical
意思是：
只有日志等级 ≥ debug 的消息才会输出到终端*/
  logger_ = std::make_shared<spdlog::logger>(
      "", spdlog::sinks_init_list{file_sink, console_sink});
  //上面2行是创建logger对象，第一个参数 "" 是 logger 名称（空字符串也可以）
  /*第二个参数是 sink 列表，日志会同时输出到：文件和控制台（终端）*/ 
  logger_->set_level(spdlog::level::debug);//只记录 debug 及以上日志
  logger_->flush_on(spdlog::level::info);//info 级别及以上立即刷新输出。当日志级别 ≥ info 时，会立刻 flush（写入磁盘）
}

std::shared_ptr<spdlog::logger> logger() {
  if (!logger_)
    set_logger();
  return logger_;
}
/*上面的意思是：如果 logger_ 为空（第一次调用），调用 set_logger() 初始化
否则直接返回已有 logger_*/
} // namespace tools
