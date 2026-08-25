    i=0.1;
    j=1;
    thetag=zeros(1,40);
    k=0.1:0.005:0.295;
    while i<=0.3
        thetag(j) = double(get_thetag(i));
        i=i+0.005;
        j=j+1;
    end
    ak=polyfit(k,thetag,4);

    [ak,s]=polyfit(k,thetag,4);
    thetagf=polyval(ak,k);
    figure;
    plot(k,thetagf,k,thetag,'o');xlabel('L0');ylabel('thetag');legend('real','fitting');
    title('thetag quadratic fit')