close all;
clear;clc;
%% 
global l_1 l_2 l_3 l_4 l_5 phi_1 phi_4;
l_1=0.15;
l_2=0.27;
l_3=0.27;
l_4=0.15;
l_5=0.15;
t=90:0.1:270;
phi=zeros(size(t));
L0=zeros(size(t));

j=1;
for i=t
    phi_1=i/180*pi;
    phi_4=(180-i)/180*pi;

    phi(j)=180-i;
    L0(j)=((l_2*sin(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) + l_1*sin(phi_1))^2 + (l_2*cos(2*atan(((4*l_2^2*(l_1*sin(phi_1) - l_4*sin(phi_4))^2 - ((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2)^2 + 4*l_2^2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2)^(1/2) - 2*l_2*(l_1*sin(phi_1) - l_4*sin(phi_4)))/((l_1*sin(phi_1) - l_4*sin(phi_4))^2 + (l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))^2 + l_2^2 - l_3^2 + 2*l_2*(l_5 - l_1*cos(phi_1) + l_4*cos(phi_4))))) - l_5/2 + l_1*cos(phi_1))^2)^(1/2);
    L0(j)=L0(j)*100;
    j=j+1;
end
%% 
figure(1);
plot(phi,L0);
figure(2);
plot(phi,L0);
axis([-90 0 9 15]);
