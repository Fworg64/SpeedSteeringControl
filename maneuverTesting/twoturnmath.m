%temper
syms yc2 yg N M xc2 xg r xm ym

%M is factor to make unit vector along normal line to goal
eqn = 2*r == sqrt((0-(xg + r*M))^2 + (r - (yg +r*N))^2)

solve(eqn,r)