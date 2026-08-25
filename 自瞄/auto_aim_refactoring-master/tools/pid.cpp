#include "pid.hpp"

#include "math_tools.hpp"

float clip(float value, float min, float max) { return std::max(min, std::min(max, value)); }
//上面是clip限幅函数，把value限制在min和max之间。太小变成min，太大变成max。
namespace tools
{
PID::PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular)
: dt_(dt), kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout), angular_(angular)
{
}//初始化列表，都是const，只能初始化一次。

float PID::calc(float set, float fdb)
{
  float e = angular_ ? limit_rad(set - fdb) : (set - fdb);//角度模式下，误差会做 [-pi, pi] wrap，避免角度跳变
  float de = angular_ ? limit_rad(last_fdb_ - fdb) : (last_fdb_ - fdb);//再算变化量de,用的是反馈值的差分而非误差差分，角度模式下，也是限制在正负180度以内
  last_fdb_ = fdb;//更新反馈值

  this->pout = e * kp_;
  this->iout = clip(this->iout + e * dt_ * ki_, -max_iout_, max_iout_);
  this->dout = de / dt_ * kd_;//算PID三项输出

  return clip(this->pout + this->iout + this->dout, -max_out_, max_out_);//最终输出，返回限幅后的输出
}

}  // namespace tools
