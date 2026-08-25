#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;//默认不退出
bool exiter_inited_ = false;//默认未初始化。防止创建多个对象。

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");//防止创建多个对象
  std::signal(SIGINT, [](int) { exit_ = true; });//按下ctrl+c的时候，停止程序
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools