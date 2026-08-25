# VMC气弹簧补偿C语言版本

## 文件说明

- `vmc_spring_compensation.h` - 头文件，包含所有函数声明和数据结构
- `vmc_spring_compensation.c` - 实现文件，包含核心算法
- `vmc_example.c` - 使用示例，展示如何调用函数

## 编译方法

### Windows (MinGW/GCC)
```bash
gcc -o vmc_example vmc_example.c vmc_spring_compensation.c -lm -std=c99
```

### Linux/macOS
```bash
gcc -o vmc_example vmc_example.c vmc_spring_compensation.c -lm -std=c99
```

### 嵌入式平台（如STM32）
将 `vmc_spring_compensation.c` 和 `vmc_spring_compensation.h` 添加到你的工程中即可。

## 运行示例
```bash
./vmc_example
```

## 核心函数使用说明

### 1. 计算弹簧补偿力（最常用）

```c
#include "vmc_spring_compensation.h"

// 初始化参数
VMC_LinkageParams params = {
    .L1 = 0.21f,              // 大腿1长度 (m)
    .L2 = 0.25f,              // 小腿1长度 (m)
    .L3 = 0.25f,              // 小腿2长度 (m)
    .L4 = 0.21f,              // 大腿2长度 (m)
    .spring_length = 0.04863f, // 弹簧力臂 (m)
    .spring_force = 400.0f     // 弹簧力 (N)
};

// 设置关节角度
VMC_JointState joint = {
    .Q1 = 2.0f,  // 关节1角度 (rad)
    .Q4 = 1.0f   // 关节4角度 (rad)
};

// 计算补偿力
VMC_SpringCompensation compensation;
if (VMC_CalcSpringCompensation(&joint, &params, &compensation)) {
    // 使用补偿力
    float F_L0 = compensation.F_L0;      // 虚拟腿方向力 (N)
    float F_Cx = compensation.F_Cx;      // x方向力 (N)
    float F_Cy = compensation.F_Cy;      // y方向力 (N)
}
```

### 2. 正运动学计算

```c
VMC_VirtualLeg vleg;
if (VMC_ForwardKinematics(&joint, &params, &vleg)) {
    float L0 = vleg.L0;    // 虚拟腿长度 (m)
    float Q0 = vleg.Q0;    // 虚拟腿角度 (rad)
    float Cx = vleg.Cx;    // 足端x坐标 (m)
    float Cy = vleg.Cy;    // 足端y坐标 (m)
}
```

### 3. 关节扭矩转末端力

```c
float tau1 = 5.0f;  // 关节1扭矩 (N·m)
float tau4 = 3.0f;  // 关节4扭矩 (N·m)
float F_L0, F_Cx, F_Cy;

if (VMC_TorqueToForce(&joint, &params, tau1, tau4, &F_L0, &F_Cx, &F_Cy)) {
    // F_L0: 虚拟腿方向等效力
    // F_Cx, F_Cy: 笛卡尔坐标系等效力
}
```

## 实际控制应用示例

```c
// 1. 获取当前关节角度（从编码器读取）
VMC_JointState joint = {
    .Q1 = encoder_read_Q1(),
    .Q4 = encoder_read_Q4()
};

// 2. 计算弹簧补偿力
VMC_SpringCompensation spring_comp;
VMC_CalcSpringCompensation(&joint, &params, &spring_comp);

// 3. 计算需要的电机力
float F_desired = 100.0f;  // 期望的虚拟腿力 (N)
float F_motor = F_desired - spring_comp.F_L0;  // 扣除弹簧补偿

// 4. 转换为关节扭矩（需要雅可比矩阵，见示例4）
// tau1 = F_motor * dL0/dQ1
// tau4 = F_motor * dL0/dQ4

// 5. 发送扭矩指令到电机
motor_set_torque(MOTOR_1, tau1);
motor_set_torque(MOTOR_4, tau4);
```

## 数学原理

### 虚功原理
弹簧做功 = 虚拟腿做功

```
F_spring × δ(S·AD) = F_L0 × δL0
```

其中：
- `S` 是弹簧作用点（在DC杆上距D点spring_length处）
- `S·AD` 是S点在AD方向的投影
- `F_L0` 是虚拟腿方向的等效力

### 雅可比转置
关节扭矩到末端力的映射：

```
τ = J^T × F
```

求解：
```
F_L0 = (τ1×∂L0/∂Q1 + τ4×∂L0/∂Q4) / (∂L0/∂Q1² + ∂L0/∂Q4²)
```

## 参数说明

### 连杆参数（VMC_LinkageParams）
- `L1`: 大腿1长度，通常0.15-0.30m
- `L2`: 小腿1长度，通常0.20-0.35m
- `L3`: 小腿2长度，通常等于L2
- `L4`: 大腿2长度，通常等于L1
- `spring_length`: 弹簧力臂长度，通常0.03-0.08m
- `spring_force`: 弹簧力大小，根据实际气弹簧测量

### 关节角度（VMC_JointState）
- `Q1`: 关节1角度，单位弧度，范围通常0-π
- `Q4`: 关节4角度，单位弧度，范围通常0-π

### 注意事项
1. 角度单位为**弧度**，不是角度
2. 坐标系：x轴水平向右，y轴竖直向下
3. 函数返回值：1表示成功，0表示失败
4. 失败原因通常是配置超出机构可达范围

## 性能优化建议

1. **减少重复计算**：如果关节角度不变，不需要重复调用
2. **查表法**：可以预先计算常用角度的补偿力，建立查找表
3. **定点运算**：嵌入式平台可以考虑将浮点转为定点数
4. **简化模型**：如果L1=L4且L2=L3，可以使用对称简化公式

## 常见问题

**Q: 函数返回0是什么原因？**
A: 通常是关节角度配置导致机构无解，检查Q1和Q4是否在合理范围内。

**Q: 补偿力为负数正常吗？**
A: 正常。负数表示弹簧力方向与虚拟腿方向相反（压缩/拉伸）。

**Q: 如何确定spring_length和spring_force？**
A: 
- spring_length：测量弹簧安装点到D点的距离
- spring_force：使用测力计测量弹簧在工作位置的力

**Q: 计算精度如何？**
A: 使用float类型，精度约6-7位有效数字，对于机器人控制足够。如需更高精度可改为double。

## 移植到其他平台

### Arduino
```cpp
// 包含math.h
#include <math.h>
#include "vmc_spring_compensation.h"

void setup() {
    Serial.begin(115200);
}

void loop() {
    VMC_JointState joint = {2.0, 1.0};
    VMC_SpringCompensation comp;
    
    if (VMC_CalcSpringCompensation(&joint, &params, &comp)) {
        Serial.print("F_L0: ");
        Serial.println(comp.F_L0);
    }
    delay(100);
}
```

### STM32 HAL
```c
// 在main.c中包含
#include "vmc_spring_compensation.h"

// 在定时器中断或主循环中调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {  // 1kHz控制频率
        VMC_JointState joint = {
            .Q1 = read_encoder_1(),
            .Q4 = read_encoder_4()
        };
        
        VMC_SpringCompensation comp;
        VMC_CalcSpringCompensation(&joint, &params, &comp);
        
        // 使用补偿力进行控制
        control_motor(comp.F_L0);
    }
}
```

## 许可证

本代码基于Python VMC Visualizer生成，可自由用于学习和商业项目。
