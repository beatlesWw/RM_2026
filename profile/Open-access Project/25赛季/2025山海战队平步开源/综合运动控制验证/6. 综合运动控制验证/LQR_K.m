close all;
clear;clc;
%% 参数
global g R L L_M l M m_p m_w I_p I_M I_w;
L0=0.15;
g=9.80665;
R=0.155/2;
L=L0/2;
L_M=L0/2;
l=0.02;
M=10;
m_p=1;
m_w=1;
I_p=1/12*m_p*(L+L_M)^2;
I_M=1/12*M*(0.1^2+0.3^2);
I_w=1/2*m_w*R^2;

A21=(I_M*I_w*L*g*m_p+I_M*L*M^2*R^2*g+I_M*L_M*M^2*R^2*g+I_w*L*M^2*g*l^2+I_w*L_M*M^2*g*l^2+I_M*L*R^2*g*m_p^2+I_M*I_w*L*M*g+I_M*I_w*L_M*M*g+L*M*R^2*g*l^2*m_p^2+L*M^2*R^2*g*l^2*m_p+L*M^2*R^2*g*l^2*m_w+L_M*M^2*R^2*g*l^2*m_p+L_M*M^2*R^2*g*l^2*m_w+2*I_M*L*M*R^2*g*m_p+I_M*L*M*R^2*g*m_w+I_M*L_M*M*R^2*g*m_p+I_M*L_M*M*R^2*g*m_w+I_w*L*M*g*l^2*m_p+I_M*L*R^2*g*m_p*m_w+L*M*R^2*g*l^2*m_p*m_w)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
A25=(I_w*L*M^2*g*l^2+I_w*L_M*M^2*g*l^2+L*M^2*R^2*g*l^2*m_w+L_M*M^2*R^2*g*l^2*m_p+L_M*M^2*R^2*g*l^2*m_w)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
A41=-(R*(R*g*L^2*M^2*l^2*m_p+I_M*R*g*L^2*M^2+R*g*L^2*M*l^2*m_p^2+2*I_M*R*g*L^2*M*m_p+I_M*R*g*L^2*m_p^2+R*g*L*L_M*M^2*l^2*m_p+2*I_M*R*g*L*L_M*M^2+2*I_M*R*g*L*L_M*M*m_p+I_M*R*g*L_M^2*M^2))/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
A45=(R*(I_p*M^2*R*g*l^2-L*L_M*M^2*R*g*l^2*m_p))/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
A61=(g*l*m_w*L^2*M^2*R^2+I_w*g*l*L^2*M^2+g*l*m_w*L^2*M*R^2*m_p+I_w*g*l*L^2*M*m_p+g*l*L*L_M*M^2*R^2*m_p+2*g*l*m_w*L*L_M*M^2*R^2+2*I_w*g*l*L*L_M*M^2+g*l*L*L_M*M*R^2*m_p^2+g*l*m_w*L*L_M*M*R^2*m_p+I_w*g*l*L*L_M*M*m_p+g*l*L_M^2*M^2*R^2*m_p+g*l*m_w*L_M^2*M^2*R^2+I_w*g*l*L_M^2*M^2)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
A65=(I_p*I_w*M*g*l+I_w*L^2*M^2*g*l+I_w*L_M^2*M^2*g*l+I_p*M^2*R^2*g*l+L^2*M^2*R^2*g*l*m_w+L_M^2*M^2*R^2*g*l*m_p+L_M^2*M^2*R^2*g*l*m_w+2*I_w*L*L_M*M^2*g*l+I_w*L^2*M*g*l*m_p+I_p*M*R^2*g*l*m_p+I_p*M*R^2*g*l*m_w+2*L*L_M*M^2*R^2*g*l*m_w+L^2*M*R^2*g*l*m_p*m_w)/(I_M*I_p*I_w + I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
A=[ 0   1   0   0   0   0;
    A21 0   0   0   A25 0;
    0   0   0   1   0   0;
    A41 0   0   0   A45 0;
    0   0   0   0   0   1;
    A61 0   0   0   A65 0];

B21=-(I_M*I_w+I_M*M*R^2+I_w*M*l^2+I_M*R^2*m_p+I_M*R^2*m_w+I_M*L*M*R+I_M*L_M*M*R+M*R^2*l^2*m_p+M*R^2*l^2*m_w+I_M*L*R*m_p+L*M*R*l^2*m_p)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
B22=(I_M*I_w+I_M*M*R^2+I_w*M*l^2+I_M*R^2*m_p+I_M*R^2*m_w+M*R^2*l^2*m_p+M*R^2*l^2*m_w+I_w*L*M*l+I_w*L_M*M*l+L*M*R^2*l*m_w+L_M*M*R^2*l*m_p+L_M*M*R^2*l*m_w)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
B41=(R*(I_M*I_p+I_M*L^2*M+I_M*L_M^2*M+I_p*M*l^2+I_M*L^2*m_p+2*I_M*L*L_M*M+L^2*M*l^2*m_p+I_M*L*M*R+I_M*L_M*M*R+I_M*L*R*m_p+L*M*R*l^2*m_p))/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
B42=-(R*(I_M*L*M*R+I_M*L_M*M*R-I_p*M*R*l+I_M*L*R*m_p+L*M*R*l^2*m_p+L*L_M*M*R*l*m_p))/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
B61=-(I_w*L*M*l+I_w*L_M*M*l-I_p*M*R*l+L*M*R^2*l*m_w+L_M*M*R^2*l*m_p+L_M*M*R^2*l*m_w+L*L_M*M*R*l*m_p)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
B62=(I_p*I_w+I_w*L^2*M+I_w*L_M^2*M+I_p*M*R^2+I_w*L^2*m_p+I_p*R^2*m_p+I_p*R^2*m_w+L^2*M*R^2*m_w+L_M^2*M*R^2*m_p+L_M^2*M*R^2*m_w+2*I_w*L*L_M*M+L^2*R^2*m_p*m_w+I_w*L*M*l+I_w*L_M*M*l+2*L*L_M*M*R^2*m_w+L*M*R^2*l*m_w+L_M*M*R^2*l*m_p+L_M*M*R^2*l*m_w)/(I_M*I_p*I_w+I_p*I_w*M*l^2+I_M*I_w*L^2*m_p+I_M*I_p*R^2*m_p+I_M*I_p*R^2*m_w+I_M*I_w*L^2*M+I_M*I_w*L_M^2*M+I_M*I_p*M*R^2+I_M*L^2*M*R^2*m_w+I_M*L_M^2*M*R^2*m_p+I_M*L_M^2*M*R^2*m_w+2*I_M*I_w*L*L_M*M+I_w*L^2*M*l^2*m_p+I_p*M*R^2*l^2*m_p+I_p*M*R^2*l^2*m_w+I_M*L^2*R^2*m_p*m_w+2*I_M*L*L_M*M*R^2*m_w+L^2*M*R^2*l^2*m_p*m_w);
B=[ 0   0;
    B21 B22;
    0   0;
    B41 B42;
    0   0;
    B61 B62];

C=[ 1   0   0   0   0   0;
    0   1   0   0   0   0;
    0   0   1   0   0   0;
    0   0   0   1   0   0;
    0   0   0   0   1   0;
    0   0   0   0   0   1];
D=[ 0   0;
    0   0;
    0   0;
    0   0;
    0   0;
    0   0];

Q=[ 1   0   0   0   0   0;
    0   1   0   0   0   0;
    0   0   20   0   0   0;
    0   0   0   5   0   0;
    0   0   0   0   50   0;
    0   0   0   0   0   1];
R_=[1   0;
    0   0.25];
%% 
Qc=ctrb(A,B);
rank(Qc);
%% 计算增益矩阵
[K,S,P]=lqr(A,B,Q,R_);
K
%% 模拟系统响应
% figure(1);
% x0=[10/180*pi;
%     0;
%     0;
%     0;
%     10/180*pi;
%     0];%初始状态
% t=0:0.05:10;
% u=[ zeros(size(t));
%     zeros(size(t));];
% [y,x]=lsim(A-B*K,B,C,D,u,t,x0);%模拟系统响应
% plot(t,y);
% legend('θ(t)','dθ(t)','x(t)','dx(t)','ɸ','dɸ')%添加图例