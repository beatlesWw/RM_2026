% %1
% figure
% plot(Fn_filter.time,Fn_filter.signals(1).values,'LineWidth',3)
% hold on 
% plot(Fn_filter.time,Fn_filter.signals(2).values,'LineWidth',3)
% plot([3,5],[55,55],'--','Color',[0 0 0],'LineWidth',2);
% xlim([3,5]);
% ylim([30,130]);
% xlabel('Time (s)','FontSize',25);ylabel('Ground support force after filtering  Fnf (N)','FontSize',25);
% legend('Fnf_L','Fnf_R','Boundary','FontSize',15);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %2
% figure
% plot(Fn_filter.time,Fn_filter.signals(1).values,'LineWidth',3)
% hold on 
% plot(Fn_filter.time,Fn_filter.signals(2).values,'LineWidth',3)
% plot([5,7],[55,55],'--','Color',[0 0 0],'LineWidth',2);
% xlim([5,7]);
% ylim([30,130]);
% xlabel('Time (s)','FontSize',25);ylabel('Ground support force after filtering  Fnf (N)','FontSize',25);
% legend('Fnf_L','Fnf_R','Boundary','FontSize',15);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %3
% figure
% plot(Fn.time,Fn.signals(1).values,'LineWidth',2.5)
% hold on 
% plot(Fn.time,Fn.signals(2).values,'LineWidth',2.5)
% plot([5,7],[55,55],'--','Color',[0 0 0],'LineWidth',2);
% xlim([5,7]);
% ylim([30,130]);
% xlabel('Time (s)','FontSize',25);ylabel('Ground support force Fn (N)','FontSize',25);
% legend('Fn_L','Fn_R','Boundary','FontSize',15);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %4
% figure
% plot(Fn.time,Fn.signals(1).values,'LineWidth',2.5)
% hold on 
% plot(Fn.time,Fn.signals(2).values,'LineWidth',2.5)
% plot([3,5],[55,55],'--','Color',[0 0 0],'LineWidth',2);
% xlim([3,5]);
% ylim([30,130]);
% xlabel('Time/s','FontSize',25);ylabel('Ground support force Fn (N)','FontSize',25);
% legend('Fn_L','Fn_R','Boundary','FontSize',15);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

%5
% figure
% plot(wheel_T.time,wheel_T.signals(1).values,'LineWidth',2.5);
% hold on 
% plot(wheel_T.time,wheel_T.signals(2).values,'LineWidth',2.5);
% plot([0,14],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,14]);
% ylim([-6,6]);
% xlabel('Time (s)','FontSize',25);ylabel('The output torque of the wheels T (Nm)','FontSize',25);
% legend('T_L','T_R','FontSize',20);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off
% 
% %6
% figure
% plot(joint_T.time,joint_T.signals(1).values,'LineWidth',2.5);
% hold on 
% plot(joint_T.time,joint_T.signals(2).values,'LineWidth',2.5);
% plot([0,14],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,14]);
% ylim([-50,50]);
% xlabel('Time (s)','FontSize',25);ylabel('The output torque of the joints Tj (Nm)','FontSize',25);
% legend('Tj_L','Tj_R','FontSize',20);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %7
% figure
% plot(thetap.time,thetap.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,20]);
% ylim([-10,25]);
% xlabel('Time (s)','FontSize',25);ylabel('The pitch Angle of the body \theta_p (deg)','FontSize',25);
% legend('\theta_p','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% 8
% figure
% plot(d_thetap.time,d_thetap.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,20]);
% ylim([-60,80]);
% xlabel('Time (s)','FontSize',25);ylabel('The angular velocity of the body  ${\dot{\theta} }_p$ (deg/s)','Interpreter','latex','FontSize',25);
% legend('${\dot{\theta} }_p$ ','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %9
% figure
% plot(v.time,v.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,20]);
% ylim([-1,3]);
% xlabel('Time (s)','FontSize',25);ylabel('wheel speed  ${\dot{x} }_w$(m/s)','Interpreter','latex','FontSize',25);
% legend('${\dot{x} }_w$','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %10
% figure
% plot(x.time,x.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,20]);
% ylim([-1,10]);
% xlabel('Time (s)','FontSize',25);ylabel('Displacement of wheel $x_w$ (m)','Interpreter','latex','FontSize',25);
% legend('$x_w$','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %11
% figure
% plot(roll_angle.time,roll_angle.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,20]);
% ylim([-8,6]);
% xlabel('Time (s)','FontSize',25);ylabel('Body roll Angle $\theta_r$ (rad)','Interpreter','latex','FontSize',25);
% legend('$\theta_r$','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %12
% figure
% plot(L0.time,L0.signals(1).values,'LineWidth',2.5);
% hold on 
% plot(L0.time,L0.signals(2).values,'LineWidth',2.5);
% plot([0,20],[0.15,0.15],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,20]);
% ylim([0,0.3]);
% xlabel('Time (s)','FontSize',25);ylabel('Leg length  L_0 (m)','FontSize',25);
% legend('L_0l','L_0r','FontSize',15);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

figure
plot(leg,k11,'o',x0,y11,'r','LineWidth',2.5,'MarkerSize',8);xlabel('equivalent pendulum length  L_p (m)','FontSize',20);ylabel('gain matrix K : k11','FontSize',30);title('k11','FontSize',25);legend('real','fitting','FontSize',45);
set(gca,'looseInset',[0 0 0 0],'LineWidth',2.5,'FontName','Times New Roman','FontSize',38);%去掉白色边框
figure
plot(leg,k12,'o',x0,y12,'r','LineWidth',2.5,'MarkerSize',8);xlabel('equivalent pendulum length  L_p (m)','FontSize',20);ylabel('gain matrix K : k12','FontSize',30);title('k12','FontSize',25);legend('real','fitting','FontSize',45);
set(gca,'looseInset',[0 0 0 0],'LineWidth',2.5,'FontName','Times New Roman','FontSize',38);%去掉白色边框
figure
plot(leg,k13,'o',x0,y13,'r','LineWidth',2.5,'MarkerSize',8);xlabel('equivalent pendulum length  L_p (m)','FontSize',25);ylabel('gain matrix K : k13','FontSize',30);title('k13','FontSize',30);legend('real','fitting','FontSize',45);
set(gca,'looseInset',[0 0 0 0],'LineWidth',2.5,'FontName','Times New Roman','FontSize',38);%去掉白色边框
figure
plot(leg,k14,'o',x0,y14,'r','LineWidth',2.5,'MarkerSize',8);xlabel('equivalent pendulum length  L_p (m)','FontSize',20);ylabel('gain matrix K : k14','FontSize',30);title('k14','FontSize',25);legend('real','fitting','FontSize',45);
set(gca,'looseInset',[0 0 0 0],'LineWidth',2.5,'FontName','Times New Roman','FontSize',38);%去掉白色边框

% figure
% plot(zero.time,zero.signals.values(:,:),'LineWidth',2.5);
% hold on 
% plot(one.time,one.signals.values(:,:),'LineWidth',2.5);
% plot(half_one.time,half_one.signals.values(:,:),'LineWidth',2.5);
% plot(two.time,two.signals.values(:,:),'LineWidth',2.5);
% plot(two_half.time,two_half.signals.values(:,:),'LineWidth',2.5);
% plot([0,10],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% title('effect of mass bias on \theta_p (deg)','FontSize',15);
% xlabel('Time (s)','FontSize',15);ylabel('The pitch Angle of the body \it\theta_p (deg)','FontSize',15);
% legend('0$\mathrm{kg}$','1$\mathrm{kg}$','1.5$\mathrm{kg}$','2$\mathrm{kg}$','2.5$\mathrm{kg}$','Interpreter','latex','FontSize',12);
% set(gca,'looseInset',[0 0 0 0],'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off
