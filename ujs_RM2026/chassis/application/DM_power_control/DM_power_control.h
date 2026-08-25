#ifndef APPLICATION_DM_POWER_CONTROL_H
#define APPLICATION_DM_POWER_CONTROL_H

#include <stdint.h>

typedef struct
{
    float k_speed;
    float k_torque_square;
    float k_mechanical;
    float constant_loss;
    float max_power;
    float min_power;
    float max_output;
    float filter_alpha;
} AppPowerControlConfig_t;

typedef struct
{
    float left_power;
    float right_power;
    float total_power;
    float limited_total_power;
    float left_scale;
    float right_scale;
    float common_scale;
    float max_power;
    uint8_t limited;
} AppPowerControlState_t;

void AppPowerControl_Init(const AppPowerControlConfig_t *config);
void AppPowerControl_SetMaxPower(float max_power);
void AppPowerControl_SetModel(float k_speed, float k_torque_square, float k_mechanical, float constant_loss);
void AppPowerControl_Reset(void);
void AppPowerControl_LimitTwoMotor(float left_speed, float right_speed, float *left_output, float *right_output);
const AppPowerControlState_t *AppPowerControl_GetState(void);
float AppPowerControl_EstimateSinglePower(float speed, float output);

#endif
