# xing_change_sp2025
## 改版sp_2025代码
## 2025.11.8日更新
## 改动：
## 1、将can接口都改成了usb接口，主要在：
## 利用代码自带的：./io/serial  串口，将can进行更改。
## 最大的变化是:./io/cboard.cpp和./io/cboard.hpp部分。
## ./configs大致更改了一些：
***
#####-----cboard参数-----#####
quaternion_canid: 0x100
bullet_speed_canid: 0x101
send_canid: 0xff
# can_interface: "can0" （被我注释掉了）
serial_port: "/tmp/ttyV0"  # 串口设备路径
baudrate: 115200             # 波特率（需要与下位机匹配）
***
## 主要也是进行了如下更改，并且加以注释
## 2、
## 但是仍然存在问题：
## 是硬件层面的，应该发现了串口设备是虚拟的，且其中的cboard.cpp与cboard.hpp代码中不可避免地使用了假数据。这是缺少下位机导致的。
## 今天下午3点后，会与沙同学交界，来尝试串口通信。
