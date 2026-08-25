#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
/*预测目标下一帧位置（predict）
用当前检测到的装甲板观测值更新估计（update）
判断目标是否“收敛/发散”（convergened / diverged）
根据 EKF 输出，反推每个装甲板的位置（armor_xyza_list）*/
namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),//装甲板名称
  armor_type(armor.type),//装甲板类型
  jumped(false),//是否跳步
  last_id(0),//最后ID
  update_count_(0),//更新次数
  armor_num_(armor_num),//装甲板数量
  t_(t),//时间
  is_switch_(false),//是否切换
  is_converged_(false),//是否收敛
  switch_count_(0)//切换次数
{
  auto r = radius;//半径
  priority = armor.priority;//优先级
  const Eigen::VectorXd & xyz = armor.xyz_in_world;//世界坐标
  const Eigen::VectorXd & ypr = armor.ypr_in_world;//偏航角、俯仰角、滚转角

  // 旋转中心的坐标——把“装甲板中心”反推回“旋转中心（圆心）”
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];
  /*xyz：这是装甲板的世界坐标（装甲板中心）
  ypr[0]：装甲板的 yaw（朝向角）
  r：旋转半径（装甲板到旋转中心的距离*/
  // x vx y vy z vz a w r l h
  // a: angle  角度 （单位：弧度）
  // w: angular velocity  角速度
  // l: r2 - r1 （长半轴差）【装甲板中心点的运动轨迹是一个椭圆】
  // h: z2 - z1 （高度差）
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();//初始化预测量协方差

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;//限制在正负180度的闭区间内，保证求和正确
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;//限制在正负180度的闭区间内，保证求和正确
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}
//下面那段是在时间间隔 dt 内预测目标的状态（位置、速度、角度、角速度等）
void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  /*下面是典型的“匀加速模型（constant velocity）” 的 F 矩阵
  你可以理解成：
  位置 = 位置 + 速度 * dt
  速度 = 速度（不变）
  角度 = 角度 + 角速度 * dt
  角速度 = 角速度（不变）
  所以：
  x'  = x + vx*dt
  vx' = vx
  y'  = y + vy*dt
  vy' = vy
  z'  = z + vz*dt
  vz' = vz
  yaw' = yaw + yaw_rate*dt
  yaw_rate' = yaw_rate*/
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;
    // v2 = 600;  // 角加速度方差（原来400，提高以改善跟踪性能）——原来400
  }//前哨站运动更平稳，所以加速度方差更小，其他装甲板（比如移动装甲）运动更不稳定，所以噪声更大
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差Q
  // clang-format off
  /*匀加速模型对应的过程噪声。但是真实目标不是严格匀速的，它会有随机加速度。
这个随机加速度会引入预测误差，Q 就是“模型误差”的协方差矩阵*/
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);//最终预测调用
}//这是一个经验阈值，防止角速度估计跑飞。
/*下面是从当前检测到的多个装甲板中，选出“最可能属于当前目标”的那一个*/
void Target::update(const Armor & armor)
{
  // 装甲板匹配
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();//生成预测的装甲板列表

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }// 给每个装甲板编号

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];// 给每个装甲板排序。distance（距离）越小越靠前
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }
  /*距离最近的 3 个装甲板，在这 3 个里比较 角度误差 + 俯仰误差，误差最小的就是最可能的匹配目标*/

  if (id != 0) jumped = true;//查看是否跳变

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }//查看是否切换目标

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;//如果两次不一样，就认为“切换目标装甲板”：

  update_ypda(armor, id);//最后用选中的装甲板更新 EKF
}

/*下面的函数是用 EKF 把当前帧检测到的装甲板观测值 armor 更新到目标状态 ekf_.x 中。
把每一帧的“观测值”融合成一个稳定的“目标状态”*/
void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi【雅克比矩阵】
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  //上面一行原本即注释掉了
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};//动态生成测量噪声 R（观测噪声协方差）

  //测量该函数内部的上面的过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }
//根据 EKF 估计的旋转圆心 + 角度，计算出每个装甲板的三维位置（x,y,z）和角度（angle）。
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}
//判断是否“跑偏”
bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;//半径是不是合理
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;//长轴半径是否也合理

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}
//判断是否稳定
bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
//给定一个目标（圆/椭圆）状态 x 和装甲板编号 id，计算出该装甲板中心的三维坐标（x,y,z）。
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);/*只有当 装甲板数量为 4 且 id 是 1 或 3 时才为真*/
  //只有当 装甲板数量为 4 且 id 是 1 或 3 时才为真
  /*说明目标装甲板是 4 个时
    1号和3号是“长轴方向的装甲板”（长轴/短轴不同）
    0号和2号是“短轴方向的装甲板”。短轴装甲板：半径 = r
    长轴装甲板：半径 = r + l*/
  auto r = (use_l_h) ? x[8] + x[9] : x[8];//装甲板半径
  auto armor_x = x[0] - r * std::cos(angle);//装甲板x坐标
  auto armor_y = x[2] - r * std::sin(angle);//装甲板y坐标
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];//装甲板z坐标

  return {armor_x, armor_y, armor_z};
}
//雅克比矩阵：观测函数 h(x) 对状态 x 的导数（偏导）观测量（yaw pitch distance angle）相对于状态向量 x 的敏感度
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);//当前装甲板的朝向角。
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);//是否使用长轴

  auto r = (use_l_h) ? x[8] + x[9] : x[8];//装甲板半径
  auto dx_da = r * std::sin(angle);//x方向对角度的偏导
  auto dy_da = -r * std::cos(angle);//y方向对角度的偏导

  auto dx_dr = -std::cos(angle);//x方向对半径的偏导
  auto dy_dr = -std::sin(angle);//y方向对半径的偏导
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;//x方向对长轴的偏导
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;//y方向对长轴的偏导

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;//z方向对高度的偏导

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;/*这其实是链式法则（链式求导）：h(x) = g(f(x))*/
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
