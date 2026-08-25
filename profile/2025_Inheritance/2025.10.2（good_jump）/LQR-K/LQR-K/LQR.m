
clear;

L0s=0.13:0.01:0.32; 
Ks=zeros(2,6,length(L0s)); %

for step=1:length(L0s)
    
    % 鎵?渶绗﹀彿閲?
    % bata1涓哄乏渚х數鏈鸿搴?
    syms theta theta1 theta2 ; 
    syms x x1 x2;
    syms phi phi1 phi2; 
    syms bata1 bata2;
    syms T Tp N P Nm Pm Nf t;
    
    l1=0.15; l2=0.27;  x0=0.15; 
    R=0.0755;  l=0.1595; mw=0.482; mp=1.058; M=21.25; r=sqrt((x0/2)^2+l^2);


    %eqs
    eqs=l1*sin(bata1)+l2*sin(bata2)-L0s(step)==0;
    eqs1=l1*cos(bata1)+l2*cos(bata2)+x0==0;
    [bata1,bata2]=solve(eqs,eqs1,bata1,bata2);
    bata1= max(bata1);
    bata2= min(bata2);


    %LM鎽嗘潌閲嶅績鍒版満浣撹浆杞磋窛绂?
    %                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    Lm=0.5*(2*l1*l2*sin(bata1)+l1^2*sin(bata1)+l2^2*sin(bata2))/(l1+l2);
    %L鎽嗘潌閲嶅績鍒伴┍鍔ㄨ疆杞村績璺濈
    L=L0s(step)-Lm;


    %Iw驱动轮转子转动惯量
    Iw=0.5*mw*R*R; 
   
    %Ip鑵胯浆鍔ㄦ儻閲?
    Ip=l1*mp*((Lm-0.5*l1*sin(bata1)^2)+(x0-0.5*l1*cos(bata1))^2)/(l1+l2)+l2*((L-0.5*l2*sin(bata2))^2+(l2*cos(bata2)))/(l1+l2);      
    %鏈轰綋杞姩鎯噺
    Im=M*r*r;




    g=9.8;
    % 杩涜鐗╃悊璁＄畻
    Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
    Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));

    %灏哊M PM N P涓棿鍙橀噺绾︽帀
    equ1=x2-(T-N*R)/(Iw/R+mw*R);
    equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
    equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
    [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
    
    % 姹傚緱闆呭厠姣旂煩闃碉紝鐒跺悗寰楀埌鐘舵?绌洪棿鏂圭▼
    Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
    Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
%     vpa(Ja)  //精度
%     vpa(Jb)
    A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
%     vpa(A)
%     vpa(B)

    [G,H]=c2d(eval(A),eval(B),0.005);
    
% Q=diag([18 0.1 80 110 700 1]);
% Q=diag([30 2 60 400 4000 1]);

%  Q=diag([5 1 10 50 600 1]);
% 
%     R=diag([1 0.25]);
%状态变量为 theat d_theat Xb Vb phi d_phi

%  Q=diag([5 1 20 55 710 1]);去常大之前
%常用
%  Q=diag([5 1 30 70 700 1]);
%  R=[1 0;0 0.20]; 

% Q=diag([10 1 50 100 700 1]);
% R=[2.75 00 0.25]; 

%  Q=diag([5 1 60 20 700 1]);
%  R=[1 0;0 0.15]; 
%状态变量为 theat d_theat Xb Vb phi d_phi
%新云台
 Q=diag([5 1 20 100  600 1]);
 R=[1 0;0 0.20]; 

%     Q=diag([2 0.1 3 25 3000 0.1]);
%     R=diag([1 1]);


%不抖但是刹不住
% Q=diag([15 1 80 200 800 1]);
% R=[60 0;0 20]; 

%  Q=diag([1 0.08 25 75 500 0.8]);
%  R=[20 0;0 5];
%  Q=diag([4 0.1 50 150 500 0.1]);
%  R=[20 0;0 5];
%  Q=diag([10 1 70 100 700 1]);
%  R=[1 0;0 0.60]; 



% Q=diag([25 1 50 100 700 1]); 
% R=[1 0;0 0.5];
%不抖


%     Q=diag([15 1 50 100 500 1]);
%     R=diag([1 0.25]);






    Ks(:,:,step)=dlqr(G,H,Q,R);

end

% 瀵筀鐨勬瘡涓厓绱犲叧浜嶭0杩涜鎷熷悎
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

% 杈撳嚭鍒癿鍑芥暟
matlabFunction(K,'File','LQR_K');

% 浠ｅ叆L0=0.2鎵撳嵃鐭╅樀K锛屽惎鍔ㄤ綅缃殑鍒濆K
%  vpa(subs(K,L0,0.2))
% 
% 
% disp("LQR妯″瀷鍑芥暟鐢熸垚瀹屾瘯");