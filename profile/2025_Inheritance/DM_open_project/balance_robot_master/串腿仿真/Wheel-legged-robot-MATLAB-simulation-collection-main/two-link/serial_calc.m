syms l1 l2 xc yc theta1 theta2 l0 phi0 d_phi0 d_theta1 d_theta2

xc = l1*cos(theta1) + l2*cos(theta2);
yc = l1*sin(theta1) + l2*sin(theta2);
l0 = sqrt(xc^2+yc^2);
phi0 = atan(yc/xc);
J=simplify(jacobian([l0;phi0],[theta1;theta2]));
J(1,1);
J(1,2);
J(2,1);
J(2,2);
d_phi0 = J(2,1)*d_theta1 + J(2,2)*d_theta2
