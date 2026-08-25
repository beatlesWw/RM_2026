#ifndef __STP_H
#define __STP_H

#include "stdint.h"

typedef struct
{
	uint16_t distance;
	uint8_t confidence;
}STP_MeasurePoint;

extern float STP_Distance;

void STP_Init(void);

#endif
