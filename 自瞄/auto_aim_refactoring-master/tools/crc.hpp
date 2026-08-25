#ifndef TOOLS__CRC_HPP
#define TOOLS__CRC_HPP

#include <cstdint>

namespace tools
{
// len不包括crc8『生成校验码』
uint8_t get_crc8(const uint8_t * data, uint16_t len);

// len包括crc8『验证』
bool check_crc8(const uint8_t * data, uint16_t len);

// len不包括crc16
uint16_t get_crc16(const uint8_t * data, uint32_t len);

// len包括crc16
bool check_crc16(const uint8_t * data, uint32_t len);

}  // namespace tools

#endif  // TOOLS__CRC_HPP

/*crc——循环冗余校验
1. CRC8 相关函数
    get_crc8(): 计算数据的CRC8校验值
        data: 需要校验的数据指针
        len: 数据长度（不包括CRC8部分）
        返回值: 计算出的CRC8值（1字节）

    check_crc8(): 验证数据的CRC8是否正确
        data: 包含CRC8的完整数据指针
        len: 数据总长度（包括CRC8部分）
        返回值: true表示校验通过，false表示校验失败

2. CRC16 相关函数

    get_crc16(): 计算数据的CRC16校验值
        data: 需要校验的数据指针
        len: 数据长度（不包括CRC16部分）
        返回值: 计算出的CRC16值（2字节）
    check_crc16(): 验证数据的CRC16是否正确
        data: 包含CRC16的完整数据指针
        len: 数据总长度（包括CRC16部分）
        返回值: true表示校验通过，false表示校验失败

uint8_t——无符号八位整数（8位，1Bite）
uint16_t——无符号八位整数（16位，2Bite）        
*/