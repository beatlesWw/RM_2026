
%%5
% figure
% plot(T.time,T.signals(1).values,'LineWidth',2.5);
% hold on 
% plot(T.time,T.signals(2).values,'LineWidth',2.5);
% plot([0,14],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,9]);
% ylim([-10,10]);
% xlabel('Time (s)','FontSize',50);ylabel('The output torque of the wheels T (N/m)','FontSize',30);
% legend('T_L','T_R','FontSize',20);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

% %6
figure
plot(L_tp.time,L_tp.signals(2).values,'LineWidth',2.5);
hold on 
plot(r_tp.time,L_tp.signals(1).values,'LineWidth',2.5);
plot(L_tp.time,L_tp.signals(1).values,'LineWidth',2.5,'LineStyle','--');
plot(r_tp.time,L_tp.signals(2).values,'LineWidth',2.5,'LineStyle','--');
plot([0,14],[0,0],'--','Color',[0,0,0],'LineWidth',2);
xlim([0,9]);
ylim([-50,50]);
xlabel('Time (s)','FontSize',25);ylabel('The output torque of the joints T_j (N/m)','FontSize',30);
legend('LT_j_2','RT_j_1','LT_j_1','RT_j_2','FontSize',15);
set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
hold off

% 
% %8
% figure
% plot(d_phi.time,d_phi.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,9]);
% ylim([-60,80]);
% xlabel('Time (s)','FontSize',25);ylabel('The angular velocity of the body  ${\dot{\theta} }_p$ (deg/s)','Interpreter','latex','FontSize',25,'FontWeight','bold');
% legend('${\dot{\theta} }_p$ ','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off
% 
% %9
% figure
% plot(d_x.time,d_x.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,9]);
% ylim([-1,3]);
% xlabel('Time (s)','FontSize',25);ylabel('wheel speed  ${\dot{x} }_w$(m/s)','Interpreter','latex','FontSize',25);
% legend('${\dot{x} }_w$','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off
% 
% %10
% figure
% plot(x.time,x.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,9]);
% ylim([-5,10]);
% xlabel('Time (s)','FontSize',25);ylabel('Displacement of wheel $x_w$ (m)','Interpreter','latex','FontSize',25);
% legend('$x_w$','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off
% 
% %11
% figure
% plot(roll.time,roll.signals.values(:,:),'LineWidth',3);
% hold on 
% plot([0,20],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,9]);
% ylim([-8,6]);
% xlabel('Time (s)','FontSize',25);ylabel('Body roll Angle $\theta_r$ (rad)','Interpreter','latex','FontSize',25);
% legend('$\theta_r$','Interpreter','latex','FontSize',25);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off
% 
% %12
% figure
% plot(l0.time,l0.signals(1).values,'LineWidth',2.5);
% hold on 
% plot(l0.time,l0.signals(2).values,'LineWidth',2.5);
% plot([0,20],[0.15,0.15],'--','Color',[0,0,0],'LineWidth',2);
% xlim([0,9]);
% ylim([0,0.5]);
% xlabel('Time (s)','FontSize',25);ylabel('Leg length  \itL_0 (m)','FontSize',25);
% legend('L_0l','L_0r','FontSize',15);
% set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
% hold off

%  figure
% % plot(zero.time,-zero.signals.values(:,:),'LineWidth',2.5);
% % hold on 
% % plot(one.time,-one.signals.values(:,:),'LineWidth',2.5);
% % plot(half_one.time,-half_one.signals.values(:,:),'LineWidth',2.5);
%  plot(two_half.time,-two_half.signals.values(:,:),'LineWidth',2.5);
%  hold on 
%   plot([0,9],[0,0],'--','Color',[0,0,0],'LineWidth',2);
%  xlim([0,9]);
% % 
% % plot([0,4],[0,0],'--','Color',[0,0,0],'LineWidth',2);
% %  xlabel('Time (s)','FontSize',25);ylabel('The pitch Angle of the body \it\theta_p (deg)','FontSize',25);
%   xlabel('Time (s)','FontSize',25);ylabel('The pitch Angle of the body \theta_p (deg)','FontSize',25);
%   legend('\theta_p','FontSize',25);
% % legend('0$\mathrm{kg}$','1$\mathrm{kg}$','1.5$\mathrm{kg}$','2$\mathrm{kg}$','Interpreter','latex','FontSize',15);
%  set(gca,'looseInset',[0 0 0 0],'FontSize',12,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
%  hold off
