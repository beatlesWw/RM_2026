#include "DM_power_control.h"
#include <math.h>

#define APP_POWER_CONTROL_DEFAULT_K_SPEED 0.05f
#define APP_POWER_CONTROL_DEFAULT_K_TORQUE_SQUARE 0.20f
#define APP_POWER_CONTROL_DEFAULT_K_MECHANICAL 1.00f
#define APP_POWER_CONTROL_DEFAULT_CONSTANT_LOSS 1.50f
#define APP_POWER_CONTROL_DEFAULT_MAX_POWER 40.0f
#define APP_POWER_CONTROL_DEFAULT_MIN_POWER 0.0f
#define APP_POWER_CONTROL_DEFAULT_MAX_OUTPUT 8.0f
#define APP_POWER_CONTROL_DEFAULT_FILTER_ALPHA 1.0f
#define APP_POWER_CONTROL_EPSILON 1.0e-6f

static AppPowerControlConfig_t power_config = {
    .k_speed = APP_POWER_CONTROL_DEFAULT_K_SPEED,
    .k_torque_square = APP_POWER_CONTROL_DEFAULT_K_TORQUE_SQUARE,
    .k_mechanical = APP_POWER_CONTROL_DEFAULT_K_MECHANICAL,
    .constant_loss = APP_POWER_CONTROL_DEFAULT_CONSTANT_LOSS,
    .max_power = APP_POWER_CONTROL_DEFAULT_MAX_POWER,
    .min_power = APP_POWER_CONTROL_DEFAULT_MIN_POWER,
    .max_output = APP_POWER_CONTROL_DEFAULT_MAX_OUTPUT,
    .filter_alpha = APP_POWER_CONTROL_DEFAULT_FILTER_ALPHA,
};

static AppPowerControlState_t power_state = {
    .left_power = 0.0f,
    .right_power = 0.0f,
    .total_power = 0.0f,
    .limited_total_power = 0.0f,
    .left_scale = 1.0f,
    .right_scale = 1.0f,
    .common_scale = 1.0f,
    .max_power = APP_POWER_CONTROL_DEFAULT_MAX_POWER,
    .limited = 0,
};

static float AppPowerControl_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static void AppPowerControl_Saturate(float *value, float min, float max)
{
    if (*value > max) {
        *value = max;
    } else if (*value < min) {
        *value = min;
    }
}

static float AppPowerControl_CalcLimitedOutput(float speed, float output, float target_power)
{
    float a = power_config.k_torque_square;
    float b = power_config.k_mechanical * speed;
    float c = power_config.k_speed * AppPowerControl_Abs(speed) + power_config.constant_loss - target_power;
    float delta = b * b - 4.0f * a * c;
    float limited_output = output;

    if (target_power <= power_config.min_power) {
        return 0.0f;
    }

    if (a <= APP_POWER_CONTROL_EPSILON || delta < 0.0f) {
        return output * sqrtf(target_power / (AppPowerControl_EstimateSinglePower(speed, output) + APP_POWER_CONTROL_EPSILON));
    }

    if (output >= 0.0f) {
        limited_output = (-b + sqrtf(delta)) / (2.0f * a);
    } else {
        limited_output = (-b - sqrtf(delta)) / (2.0f * a);
    }

    if ((output > 0.0f && limited_output < 0.0f) || (output < 0.0f && limited_output > 0.0f)) {
        limited_output = 0.0f;
    }

    return limited_output;
}

void AppPowerControl_Init(const AppPowerControlConfig_t *config)
{
    if (config != 0) {
        power_config = *config;
    }

    if (power_config.max_power < power_config.min_power) {
        power_config.max_power = power_config.min_power;
    }

    if (power_config.max_output < 0.0f) {
        power_config.max_output = -power_config.max_output;
    }

    if (power_config.filter_alpha < 0.0f) {
        power_config.filter_alpha = 0.0f;
    } else if (power_config.filter_alpha > 1.0f) {
        power_config.filter_alpha = 1.0f;
    }

    AppPowerControl_Reset();
}

void AppPowerControl_SetMaxPower(float max_power)
{
    power_config.max_power = max_power;

    if (power_config.max_power < power_config.min_power) {
        power_config.max_power = power_config.min_power;
    }
}

void AppPowerControl_SetModel(float k_speed, float k_torque_square, float k_mechanical, float constant_loss)
{
    power_config.k_speed = k_speed;
    power_config.k_torque_square = k_torque_square;
    power_config.k_mechanical = k_mechanical;
    power_config.constant_loss = constant_loss;
}

void AppPowerControl_Reset(void)
{
    power_state.left_power = 0.0f;
    power_state.right_power = 0.0f;
    power_state.total_power = 0.0f;
    power_state.limited_total_power = 0.0f;
    power_state.left_scale = 1.0f;
    power_state.right_scale = 1.0f;
    power_state.common_scale = 1.0f;
    power_state.max_power = power_config.max_power;
    power_state.limited = 0;
}

float AppPowerControl_EstimateSinglePower(float speed, float output)
{
    float power = power_config.k_mechanical * speed * output
                + power_config.k_speed * AppPowerControl_Abs(speed)
                + power_config.k_torque_square * output * output
                + power_config.constant_loss;

    if (power < power_config.min_power) {
        power = power_config.min_power;
    }

    return power;
}

void AppPowerControl_LimitTwoMotor(float left_speed, float right_speed, float *left_output, float *right_output)
{
    float left_original;
    float right_original;
    float left_limited;
    float right_limited;
    float left_target_power;
    float right_target_power;
    float available_power;
    float total_power;

    if (left_output == 0 || right_output == 0) {
        return;
    }

    AppPowerControl_Saturate(left_output, -power_config.max_output, power_config.max_output);
    AppPowerControl_Saturate(right_output, -power_config.max_output, power_config.max_output);

    left_original = *left_output;
    right_original = *right_output;
    power_state.left_power = AppPowerControl_EstimateSinglePower(left_speed, left_original);
    power_state.right_power = AppPowerControl_EstimateSinglePower(right_speed, right_original);
    total_power = power_state.left_power + power_state.right_power;

    power_state.total_power = power_config.filter_alpha * total_power + (1.0f - power_config.filter_alpha) * power_state.total_power;
    power_state.max_power = power_config.max_power;
    power_state.left_scale = 1.0f;
    power_state.right_scale = 1.0f;
    power_state.common_scale = 1.0f;
    power_state.limited = 0;

    if (total_power <= power_config.max_power || power_config.max_power <= power_config.min_power) {
        power_state.limited_total_power = total_power;
        return;
    }

    available_power = power_config.max_power;
    power_state.common_scale = available_power / (total_power + APP_POWER_CONTROL_EPSILON);
    AppPowerControl_Saturate(&power_state.common_scale, 0.0f, 1.0f);

    left_target_power = power_state.left_power * power_state.common_scale;
    right_target_power = power_state.right_power * power_state.common_scale;

    left_limited = AppPowerControl_CalcLimitedOutput(left_speed, left_original, left_target_power);
    right_limited = AppPowerControl_CalcLimitedOutput(right_speed, right_original, right_target_power);

    AppPowerControl_Saturate(&left_limited, -power_config.max_output, power_config.max_output);
    AppPowerControl_Saturate(&right_limited, -power_config.max_output, power_config.max_output);

    *left_output = left_limited;
    *right_output = right_limited;

    if (AppPowerControl_Abs(left_original) > APP_POWER_CONTROL_EPSILON) {
        power_state.left_scale = left_limited / left_original;
    } else {
        power_state.left_scale = 1.0f;
    }

    if (AppPowerControl_Abs(right_original) > APP_POWER_CONTROL_EPSILON) {
        power_state.right_scale = right_limited / right_original;
    } else {
        power_state.right_scale = 1.0f;
    }

    power_state.limited_total_power = AppPowerControl_EstimateSinglePower(left_speed, left_limited)
                                    + AppPowerControl_EstimateSinglePower(right_speed, right_limited);
    power_state.limited = 1;
}

const AppPowerControlState_t *AppPowerControl_GetState(void)
{
    return &power_state;
}
