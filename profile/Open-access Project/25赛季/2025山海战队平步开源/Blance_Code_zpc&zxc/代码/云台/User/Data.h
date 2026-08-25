#ifndef __DATA_H
#define __DATA_H

#define Data_Sqrt3			1.7320508075688772935274463415059f//√3

void Data_Clipping(float *Data,float Data_Min,float Data_Max);//数据限幅
uint8_t Data_RangeCheck(float Data,float Data_Min,float Data_Max);//数据范围检测

#endif
