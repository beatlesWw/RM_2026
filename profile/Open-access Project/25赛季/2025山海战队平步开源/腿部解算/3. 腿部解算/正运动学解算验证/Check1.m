close all;
clear;clc;
%% 
global l_1 l_2 l_3 l_4 l_5 phi_1 phi_4;
l_1=0.15;
l_2=0.27;
l_3=0.27;
l_4=0.15;
l_5=0.15;

phi_1=135/180*pi;
phi_4=45/180*pi;

L0=((l_2*sin(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) + l_1*sin(phi_1))^2 + (l_2*cos(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) - l_5/2 + l_1*cos(phi_1))^2)^(1/2)
phi_0=acos((l_2*cos(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) - l_5/2 + l_1*cos(phi_1))/((l_2*sin(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) + l_1*sin(phi_1))^2 + (l_2*cos(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) - l_5/2 + l_1*cos(phi_1))^2)^(1/2));
phi_0=phi_0/pi*180