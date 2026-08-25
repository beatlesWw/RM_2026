#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"

/*
 *函数简介:数据限幅
 *参数说明:原数据(限幅后数据也由此输出)
 *参数说明:数据最小值
 *参数说明:数据最大值
 *返回类型:无
 *备注:无
 */
void Data_Clipping(float *Data,float Data_Min,float Data_Max)
{
	if((*Data)>Data_Max)(*Data)=Data_Max;
	else if((*Data)<Data_Min)(*Data)=Data_Min;
}

/*
 *函数简介:数据范围检测
 *参数说明:检测数据
 *参数说明:数据最小值(不可取等)
 *参数说明:数据最大值(不可取等)
 *返回类型:0-不在范围内,1-在范围内
 *备注:无
 */
uint8_t Data_RangeCheck(float Data,float Data_Min,float Data_Max)
{
	if(Data_Min<Data && Data<Data_Max)return 1;
	return 0;
}
