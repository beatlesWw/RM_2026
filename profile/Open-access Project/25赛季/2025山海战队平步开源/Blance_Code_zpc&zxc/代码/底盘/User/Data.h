#ifndef __DATA_H
#define __DATA_H

#include <stdint.h>

#define Data_Sqrt3			1.7320508075688772935274463415059f	//√3
#define Data_Rad2Deg		57.295779513082320876798154814105f	//180/PI
#define Data_Deg2Rad		0.01745329251994329576923690768489f	//PI/180
#define Data_Rad2RPM		9.5492965855137201461330258023509f	//60/2PI
#define Data_RPM2Rad		0.10471975511965977461542144610932f	//2PI/60

float Data_Clipping(float Data,float Data_Min,float Data_Max);//数据限幅
uint8_t Data_RangeCheck(float Data,float Data_Min,float Data_Max);//数据范围检测

#endif
