close all;
clear;clc;
%% 参数
global g R L m_p m_w I_p I_w;
g=9.80665;
R=0.155/2;
L=0.35/2;
m_p=1;
m_w=1;
I_p=1/12*m_p*(2*L)^2;
I_w=1/2*m_w*R^2;

A21=(L*g*R^2*m_p^2 + L*g*m_w*R^2*m_p + I_w*L*g*m_p)/(I_p*I_w + I_w*L^2*m_p + I_p*R^2*m_p + I_p*R^2*m_w + L^2*R^2*m_p*m_w);
A41=-(L^2*R^2*g*m_p^2)/(I_p*I_w + I_w*L^2*m_p + I_p*R^2*m_p + I_p*R^2*m_w + L^2*R^2*m_p*m_w);
A=[ 0   1   0   0;
    A21 0   0   0;
    0   0   0   1;
    A41 0   0   0];

B21=-(I_w + R^2*m_p + R^2*m_w + L*R*m_p)/(I_p*I_w + I_w*L^2*m_p + I_p*R^2*m_p + I_p*R^2*m_w + L^2*R^2*m_p*m_w);
B41=(R*(m_p*L^2 + R*m_p*L + I_p))/(I_p*I_w + I_w*L^2*m_p + I_p*R^2*m_p + I_p*R^2*m_w + L^2*R^2*m_p*m_w);
B=[ 0;
    B21;
    0;
    B41];

C=[ 1   0   0   0;
    0   1   0   0;
    0   0   1   0;
    0   0   0   1];
D=[ 0;
    0;
    0;
    0];

Q=[ 1   0   0   0;
    0   1   0   0;
    0   0   5   0;
    0   0   0   1];
R_=1;
%% 
Qc=ctrb(A,B);
rank(Qc);
%% 计算增益矩阵
[K,S,P]=lqr(A,B,Q,R_);
K
%% 模拟系统响应
figure(1);
x0=[10/180*pi;
    0;
    0;
    0];%初始状态
t=0:0.05:10;
u=[ zeros(size(t))];
[y,x]=lsim(A-B*K,B,C,D,u,t,x0);%模拟系统响应
plot(t,y);
legend('θ(t)','dθ(t)','x(t)','dx(t)')%添加图例