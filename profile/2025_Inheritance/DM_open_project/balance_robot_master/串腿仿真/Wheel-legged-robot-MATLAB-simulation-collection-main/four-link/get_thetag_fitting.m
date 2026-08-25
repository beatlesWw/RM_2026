function akg = get_thetag_fitting()
    i=0.1;
    j=1;
    thetag=zeros(1,40);
    k=0.1:0.005:0.295;
    while i<=0.3
        thetag(j) = double(get_thetag(i));
        i=i+0.005;
        j=j+1;
    end
    akg=polyfit(k,thetag,2);
end
