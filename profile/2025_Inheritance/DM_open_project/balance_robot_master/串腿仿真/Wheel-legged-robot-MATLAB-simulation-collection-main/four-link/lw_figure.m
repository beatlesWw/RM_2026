%     i=0.1;
%     j=1;
%     k=0.1:0.005:0.295;
%     lw=zeros(1,40);
%     while i<=0.3
%         lw(j) = double(get_lw(i));
%         i=i+0.005;
%         j=j+1;
%     end
%     [ak,s]=polyfit(k,lw,2);
%     lwf=polyval(ak,k);
    figure;
    plot(k,lw,'o',k,lwf,'LineWidth',3);xlabel('L0');ylabel('lw');legend('real','fitting');
    set(gca,'looseInset',[0 0 0 0],'FontSize',20,'LineWidth',1.5,'FontName','Times New Roman');%去掉白色边框
    title('lw quadratic fit')