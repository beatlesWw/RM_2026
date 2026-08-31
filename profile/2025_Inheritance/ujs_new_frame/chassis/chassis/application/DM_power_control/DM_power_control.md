# application/DM_power_control 使用说明

## 模块目的

这个模块用于轮腿底盘中只控制左右两个足端/轮毂电机的功率限制。

它综合了 `RM2024-PowerModule-main` 的轮腿功率模型和普通四轮功率控制中的二次方程反解思路，但保留为轻量级 C 接口。

核心限制对象是左右两个电机的最终输出：

```c
left.wheel_set
right.wheel_set
```

不建议直接限制 `F0`、`Tp` 或关节 `torque_set`，因为这些量直接影响站立和腿部支撑稳定性。

## 功率模型

单个电机预计功率：

```text
P = k_mechanical * speed * output
  + k_speed * abs(speed)
  + k_torque_square * output^2
  + constant_loss
```

含义：

- `speed`：DM 电机反馈角速度，单位为 `rad/s`
- `output`：准备发送给电机的 `wheel_set`，单位为 `N.m`
- `k_mechanical * speed * output`：机械输出功率项，`rad/s * N.m = W`，所以 `k_mechanical` 默认应为 `1.0f`
- `k_speed * abs(speed)`：转速损耗项
- `k_torque_square * output^2`：力矩平方损耗项
- `constant_loss`：常值损耗

如果左右两电机预计总功率超过 `max_power`，模块会按功率比例给左右电机分配可用功率，然后通过二次方程反解新的 `output`。

## 文件

```text
application/DM_power_control/DM_power_control.h
application/DM_power_control/DM_power_control.c
application/DM_power_control/DM_power_control.md
```

注意：工程里 `modules/motor` 下已经有一个 `power_control.h`，所以本模块改名为 `DM_power_control.h`，避免和 DJI 电机功率控制模块混淆。

## 初始化

建议在 `ChassisInit()` 里初始化一次：

```c
#include "DM_power_control.h"

AppPowerControlConfig_t power_config = {
    .k_speed = 0.05f,
    .k_torque_square = 0.20f,
    .k_mechanical = 1.00f,
    .constant_loss = 1.50f,
    .max_power = 40.0f,
    .min_power = 0.0f,
    .max_output = 8.0f,
    .filter_alpha = 1.0f,
};

AppPowerControl_Init(&power_config);
```

如果暂时不想配置参数，也可以直接使用默认值：

```c
AppPowerControl_Init(0);
```

## 基本调用

在左右 `wheel_set` 都已经算完、发送电机之前调用：

```c
float left_out = left.wheel_set;
float right_out = right.wheel_set;

AppPowerControl_LimitTwoMotor(left_wheel_speed, right_wheel_speed, &left_out, &right_out);

left.wheel_set = left_out;
right.wheel_set = right_out;
```

其中 `left_wheel_speed` 和 `right_wheel_speed` 传左右 DM 轮电机反馈速度，单位为 `rad/s`。

如果你使用 DM 电机反馈，通常可以先用：

```c
W_l->measure.velocity
W_r->measure.velocity
```

你已确认该反馈速度单位为 `rad/s`，并且 `wheel_set` 单位为 `N.m`，所以机械功率项不需要额外换算。

## 推荐接入位置

推荐放在统一发送左右轮之前，而不是只放在单侧任务里。

原因：

- 功率控制需要同时知道左、右两个输出
- 左右腿任务是并行的，分别调用容易出现一边用新值、一边用旧值
- 最好在 `wheel_task` 或统一的轮电机发送位置做一次限制

如果暂时没有统一发送任务，也可以先在左右腿都算完后，由一个固定任务周期读取全局 `left.wheel_set` 和 `right.wheel_set`，限制后再发送。

## 动态设置最大功率

可以根据裁判系统或电容状态动态调整：

```c
AppPowerControl_SetMaxPower(50.0f);
```

如果没有裁判系统数据，建议先保守设置，例如：

```c
AppPowerControl_SetMaxPower(35.0f);
```

## 查看调试状态

```c
const AppPowerControlState_t *state = AppPowerControl_GetState();
```

可观察：

```c
state->left_power;
state->right_power;
state->total_power;
state->limited_total_power;
state->common_scale;
state->left_scale;
state->right_scale;
state->limited;
```

其中：

- `limited == 0`：未触发功率限制
- `limited == 1`：本周期触发了功率限制
- `common_scale`：总功率缩放比例
- `left_scale/right_scale`：左右输出实际缩放比例

## 参数调试建议

初始阶段不要直接相信默认参数，默认参数只是为了让模块能跑起来。

建议步骤：

1. 先把 `max_power` 设大，例如 `200.0f`，只观察 `state->total_power`
2. 原地扶车低速转轮，确认 `speed * wheel_set` 的符号和大小是否合理
3. 低速行走、快速加速，记录 `state->total_power` 和裁判系统实际底盘功率的差异
4. 如果空转或高速小力矩时估计偏低，增大 `k_speed`
5. 如果大力矩起步、急停时估计偏低，增大 `k_torque_square`
6. 如果整体一直偏低或偏高，微调 `constant_loss`
7. 最后逐步降低 `max_power`，观察限功率后是否仍能稳定站立

## 注意事项

- 不要在倒地自起、零力矩、轮子延时启动阶段强行使用旧的限功率输出
- 触发双腿离地保护时，应优先让 `wheel_set = 0`
- 功率控制应该在离地保护之后执行
- `max_output` 单位是 `N.m`，默认 `8.0f` 只是保守初值，需要结合你的实际轮电机能力调整
- 如果 `speed * output` 的符号长期反常，说明左右电机速度或输出方向需要统一
