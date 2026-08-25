syms Mw theta_l(t) f Px T Iw x(t) R  f1 Wx Wy Mp xp lw lp Ip d_thetap theta_p


Wx = Mp*diff(xp + lp*sin(theta_l),t,2);
Wy = Mp*g+Mp*diff(lp*cos(theta_l),t,2);
eqn1 = Ip*diff(theta_l,t,2) == Wy*lw*sin(theta_l)-Wx*lw*cos(theta_l)-T;
eqn10 = subs(eqn1,diff(theta_l,t,2),f1);
f1 = subs(subs(solve(eqn10,f1),diff(theta_l,t),d_thetap),theta_l,theta_p)