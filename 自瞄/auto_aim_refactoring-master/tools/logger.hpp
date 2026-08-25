#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>
//声明一个函数 tools::logger()，用于获取日志对象。，作用是：返回一个全局共享的 logger 实例
namespace tools
{
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP
//别的地方写日志，只需要写：tools::logger()->info("Hello");