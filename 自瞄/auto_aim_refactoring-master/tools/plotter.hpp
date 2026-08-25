#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);
  //设置默认地址位127.0.0.1:9870

  ~Plotter();

  void plot(const nlohmann::json & json); //把JSON 序列化成字符串

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_; //上锁，防止多个线程同时访问socket_
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP