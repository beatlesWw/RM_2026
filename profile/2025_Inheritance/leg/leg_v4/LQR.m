%针对机器人不同的腿长 L0，自动计算出一组最优的控制增益矩阵 K（即LQR反馈矩阵），
%并将这些增益与腿长的关系拟合为多项式函数，最终生成一个可以直接调用的函数 LQR_K(L0)
clear;
L0s=0.17:0.01:0.35; %% L0变化范围从最小到最大
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K

for step=1:length(L0s)
    
    syms theta theta1 theta2; 
    syms x x1 x2;
    syms phi phi1 phi2; 
    syms bata1 bata2;
    syms T Tp N P Nm Pm Nf t;
    
    % l1=0.13; l2=0.18;  x0=0.098;  单位m kg
    % R=0.06;  l=0.01; mw=1.13; mp=2*(0.214+0.035+0.358); M=5.5; r=sqrt((x0/2)^2+l^2); 
    
    l1=0.20;  l2=0.25;  x0=0.078; 
    R=0.151;  l=0.053; mw=0.9684; mp=1.058; M=8.5; r=sqrt((x0/2)^2+l^2);

    %eqs
    eqs=l1*sin(bata1)+l2*sin(bata2)-L0s(step)==0;
    eqs1=l1*cos(bata1)+l2*cos(bata2)+x0==0;
    [bata1,bata2]=solve(eqs,eqs1,bata1,bata2);
    bata1= max(bata1);
    bata2=min(bata2);

    %LM摆杆重心到机体转轴距离                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    Lm=0.5*(2*l1*l2*sin(bata1)+l1^2*sin(bata1)+l2^2*sin(bata2))/(l1+l2);
    %L摆杆重心到驱动轮轴距离
    L=L0s(step)-Lm;

    %Iw驱动轮转子转动惯量
    Iw=0.5*mw*R*R; 
    %Ip摆杆绕质心转动惯量
    Ip=l1*mp*((Lm-0.5*l1*sin(bata1)^2)+(x0-0.5*l1*cos(bata1))^2)/(l1+l2)+l2*((L-0.5*l2*sin(bata2))^2+(l2*cos(bata2)))/(l1+l2);      
    %Im机体绕质心转动惯量
    Im=M*r*r;
    g=9.8;
    % 对摆杆建模
    Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
    Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));

    %力矩分析
    equ1=x2-(T-N*R)/(Iw/R+mw*R); %1.12式（3）
    equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;%1.12式（6）
    equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;%1.12式（9）
    [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
    %是一个求解方程组的函数。equ1, equ2, equ3 是三个方程。x2, theta2, phi2 是待求解的变量。
     
    % 雅可比矩阵，描述变量关系
    Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
    Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
%     vpa(Ja)
%     vpa(Jb)
    %平衡点附近的矩阵
    A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
%     vpa(A)
%     vpa(B)

    % 离散化
    [G,H]=c2d(double(A),double(B),0.005);
    
    %Q, R
     % Q=diag([40 1 130 190 3400 1]);
     % R=diag([14 35]);  
     Q=diag([1 1 100 150 2000 1]);
     R=diag([25 12]);  
    %R=diag([1 0.8]);
    %算出离散化的K
    Ks(:,:,step)=dlqr(G,H,Q,R);

end

K=sym('K',[2 6]);
syms L0;
for x=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
        vpa(p(1))
        vpa(p(2))
        vpa(p(3))
        vpa(p(4))
    end
end

matlabFunction(K,'File','LQR_K');
