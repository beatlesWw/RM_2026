#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
//安全、统一地读取 YAML 配置文件，并在出错时直接终止程序。
/*提供了两个函数：
tools::load(path)加载一个 YAML 文件
tools::read<T>(yaml, key).从 YAML 中读取一个键值，并转成指定类型
目标是：配置文件错误时立刻报错不允许“默默使用默认值”*/
namespace tools
{
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);//尝试加载 YAML 文件
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);//解析错误的话退出并报错
  }
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}//读取指定键值，若不存在则报错退出

}  // namespace tools

#endif  // TOOLS__YAML_HPP