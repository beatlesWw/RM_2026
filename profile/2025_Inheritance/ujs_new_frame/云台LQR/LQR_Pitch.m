%% 简化的匀质摆杆LQR增益计算（固定杆长）
clear; clc;

% 参数设置
m = 1.0;       % 质量 (kg)
l = 0.2;       % 杆长 (m)
Ts = 0.005;    % 采样周期 (s)

% LQR权重
Q = diag([100, 10]);  % theta d_theta
R = 0.1;              % 控制权重

% 计算转动惯量
J = (1/3) * m * l^2;

% 连续系统
A = [0, 1; 0, 0];
B = [0; 1/J];

% 离散化
[Ad, Bd] = c2d(A, B, Ts);

% 计算LQR增益
K = dlqr(Ad, Bd, Q, R);

% 输出结果
fprintf('\nLQR增益：\n');
fprintf('  K = [%.6f, %.6f]\n', K(1), K(2));
fprintf('\n控制律：\n');
fprintf('  T = 0.5*m*g*l*cos(γ) - %.4f*γ - %.4f*γ_dot\n', K(1), K(2));