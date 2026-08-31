#ifndef SPRING_COMP_H
#define SPRING_COMP_H

#include <stdint.h>
#include "VMC.h"
#include "controller.h"

typedef enum {
    SPRING_SIDE_RIGHT = 0,
    SPRING_SIDE_LEFT = 1,
} SpringSide_e;

typedef struct {
    float l1;
    float l2;
    float l3;
    float l4;
    float ls;
    float s2;
    float s3;
    float leg_theta;
    float Fs;
    float Fv;
    float dt;
    float as;

} SpringCompParam_t;

extern SpringCompParam_t spring_comp_r;
extern SpringCompParam_t spring_comp_l;

float Fv_Gas_spring(vmc_leg_t *leg,SpringCompParam_t *spring);
uint8_t LegFold_Override(vmc_leg_t *vmc, float *LQR_K, float leg_tp, PIDInstance *LegPid, SpringSide_e side);
#endif
