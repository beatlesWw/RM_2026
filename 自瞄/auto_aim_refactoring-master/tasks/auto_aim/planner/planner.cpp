#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

/*做三件事：
① 预测未来目标位置
② 规划一条“理想瞄准轨迹”
③ 用 TinyMPC 算“现在这一拍云台该怎么动 + 能不能开火”*/

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}//都是读取配置文件的路径

Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. Check bullet speed——检查子弹速度
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22; //为了防止出现问题，设置一个默认值
  }

  // 1. Predict fly_time——预测飞行时间
  Eigen::Vector3d xyz;//xyz：用来存 选中的装甲板三维坐标
  auto min_dist = 1e10;//最小距离（当前找到的最小平面距离（初始化成一个很大的数））
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();//遍历所有装甲板，用“水平距离”选最近的装甲板
    if (dist < min_dist) {
      min_dist = dist;//当前目标中，最近的那块装甲板的位置
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());//构造子弹模型，计算子弹轨迹
  target.predict(bullet_traj.fly_time);//预测目标轨迹与子弹到达时期的位置。
  /*如果没有这段代码：永远在瞄 “过去的目标” 目标在横向运动时，你会永远慢半拍，看起来“跟得上”，但打不中*/

  // 2. Get trajectory
  double yaw0;//yaw0为一个参考yaw角
  Trajectory traj;//用来存一段时间窗口内的轨迹矩阵
  try {
    yaw0 = aim(target, bullet_speed)(0);// 返回一个向量
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};//无解或者无效，可以直接退出了。
  }

  // 3. Solve yaw——解算yaw角
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch——解算pitch角
  x0 << traj(2, 0), traj(3, 0);//这里用 traj 的第 3、4 行第一列作为 pitch 初始状态——拼成了一个二维向量
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);//角度正负派之间，限制了角度，不会太离谱
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;//开火判定往后看几帧偏移量，这里为2帧
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
  /*traj(...)：参考轨迹给出的“理想瞄准角”，solver_->work->x(...)：优化器算出来的“实际会执行/跟踪到的角”
  两者差的欧氏距离 < fire_thresh_ 就认为足够接近，可以开火。*/
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};// 表示“可能有目标，也可能没目标”。

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
  //如果这个量的绝对值大于阈值 decision_speed_，认为目标运动“快”，就用更大的/不同的延迟参数 high_speed_delay_time_；
  // 否则用 low_speed_delay_time_。
  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
  // 用当前时间加上 delay_time 微秒，得到一个“未来时间点” future——短暂地到达未来。

  target->predict(future);

  return plan(*target, bullet_speed);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");//读取配置文件

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};// 表示没有额外偏置项（比如重力项在这里不建模）
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);
  //上一行的意思是把这个求解器初始化为一个固定维度的问题：状态2维，控制1维，预测步长HORIZON，
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);
  //上面几行都是设置边界约束
  pitch_solver_->settings->max_iter = 10;//最多迭代10次
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;//最小距离

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    } //从目标的所有装甲板候选里，挑出“在图像/相机平面上最近的那个”，后把它的 3D 坐标 xyz 和 对应的 yaw 取出来，作为后续需要打击的装甲板
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);
  //上面意思是把当前选中的装甲板信息（x、y、z、yaw）打包成一个 4 维向量，存到 debug_xyza 里用于调试/可视化/日志输出。

  auto azim = std::atan2(xyz.y(), xyz.x());//计算目标的方位角
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());//计算子弹轨迹
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");
  //如果子弹轨迹无解，直接报错

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;//矩阵

  target.predict(-DT * (HALF_HORIZON + 1));//把目标状态回退到更早的时刻
  auto yaw_pitch_last = aim(target, bullet_speed);//计算瞄准角——因为后面算速度要用中心差分

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);//计算瞄准角，代表：“当前要写入轨迹的这一帧（center）”

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);//用中心差分算角速度

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }//主程序：每次往未来推 DT，算 next，并填充 tra

  return traj;//不断输出结果
}

}  // namespace auto_aim