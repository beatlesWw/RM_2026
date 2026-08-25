function T_joint = VMC_calc(F0,theta1)
    l1 = 0.163; 
    l2 = 0.165; 
    l3 = 0.170;
    l4 = 0.08;  
    l23 = 0.04; 

    J = l2*cos(theta1) + l1*cos(theta1 + acos((l2^2 - 2*cos(pi/4 + theta1)*l2*l4 - l3^2 + l4^2 + l23^2)/(2*l23*(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)^(1/2))) + asin((l4*sin(theta1 + pi/4))/(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)^(1/2)))*(((l4*cos(theta1 + pi/4))/(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)^(1/2) - (l2*l4^2*sin(theta1 + pi/4)^2)/(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)^(3/2))/(1 - (l4^2*sin(theta1 + pi/4)^2)/(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2))^(1/2) - ((l2*l4*sin(theta1 + pi/4))/(l23*(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)^(1/2)) - (l2*l4*sin(theta1 + pi/4)*(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 - l3^2 + l4^2 + l23^2))/(2*l23*(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)^(3/2)))/(1 - (l2^2 - 2*cos(pi/4 + theta1)*l2*l4 - l3^2 + l4^2 + l23^2)^2/(4*l23^2*(l2^2 - 2*cos(pi/4 + theta1)*l2*l4 + l4^2)))^(1/2) + 1);
    T_joint = F0*J;

end