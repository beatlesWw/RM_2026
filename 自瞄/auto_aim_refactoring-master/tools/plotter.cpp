#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)//创建一个 UDP socketUDP， 是一种“只管把数据丢出去，不管对方收没收到”的网络通信方式。
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);//实质是文本表示符
  //socket() 的作用：向操作系统申请一个“网络通信接口”，并返回一个编号（文件描述符）。
  //AF_INET是 IPv4，SOCK_DGRAM（UDP无连接、发包），返回的句柄存到 socket_

  destination_.sin_family = AF_INET;//IPv4
  destination_.sin_port = ::htons(port);//目标端口（9870）
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());//目标IP地址（127.0.0.1），是回环地址，我法给我自己。
  //根据hpp文件，默认地址是127.0.0.1:9870
}

Plotter::~Plotter() { ::close(socket_); } //析构函数，关闭socket。

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);//mutex锁来保证稳定的线程同步的进行
  auto data = json.dump();//会生成{"x":1.2,"y":3.4}的字符串，UDP 只能发字节，不能直接发 C++ 对象。（每个变量，但是不包括地址）
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    //第28行是依次是文本表示符，数据地址，长度，标志位（0），目标地址
    sizeof(destination_));//告诉地址结构体的大小
}

}  // namespace tools