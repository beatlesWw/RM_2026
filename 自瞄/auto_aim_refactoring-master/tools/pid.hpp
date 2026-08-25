#ifndef TOOLS__PID_HPP
#define TOOLS__PID_HPP

/*
P → Proportional（比例）
I → Integral（积分）
D → Derivative（微分）——其实就是 对误差做三种不同的数学处理，然后加起来生成控制输出。PID 的任务就是：把 e(t) → 控制量 u(t)，让误差归零。
P：比例（Proportional） 误差越大，输出越大。优点：响应快，缺点：误差可能永远不为零（静态偏差）
I：积分（Integral） 累积历史误差，慢慢补偿，可以消除静态偏差 缺点：累积过快 → 过冲 / 振荡、响应慢
D：微分（Derivative） 看误差变化速度，像“刹车”一样提前抑制振荡
优点：提高系统稳定性，抑制超调 缺点：对噪声敏感
| 分量 | 作用  | 典型调整思路         |
| -- | --- | -------------- |
| P  | 追误差 | 越大响应快，但容易抖     |
| I  | 消偏差 | 越大消除稳态误差，但容易过冲 |
| D  | 防抖动 | 越大抑振，但容易受噪声影响  |
P 推，I 补，D 刹

*/

namespace tools
{
class PID
{
public:
  // dt: 控制周期, 单位: s
  // kp: P项系数
  // ki: I项系数
  // kd: D项系数
  // max_out: PID最大输出值
  // max_iout I项最大输出值
  PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular = false);
  /*angular：是否角度模式（角度误差会做 [-pi, pi] wrap，避免从 +179° 到 -179° 误差算成 358° 那种问题）*/
  float pout = 0.0f;  // P项输出, 用于调试——f表示浮点数
  float iout = 0.0f;  // I项输出, 用于调试
  float dout = 0.0f;  // D项输出, 用于调试——初始化都是0.0
  //用来查看这一次计算的 P/I/D 各自贡献（方便调参时打印/画图）。
  // 计算PID输出值
  // set: 目标值
  // fdb: 反馈值(feedback)
  float calc(float set, float fdb);//输入目标直set和反馈值fdb，输出一个控制量float

private:
  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const bool angular_;//这个 PID 对象创建后，这些参数不能被修改。
 /* 为什么要“再写一次”？（看起来像重复）
因为 C++ 类通常分两层：
函数参数：构造函数里传进来的 dt, kp, ki, ... 只是“临时变量”，只在构造函数那一下存在。
成员变量：为了让对象以后每次 calc() 都能用到这些参数，必须把它们存到类里，所以要有 dt_ / kp_ / ... 这些成员。*/

  float last_fdb_ = 0.0f;  // 上次反馈值
};

}  // namespace tools

#endif  // TOOLS__PID_HPP