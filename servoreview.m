X0 = [2*pi/4;0;0];

inertiacoeff =1;

A = [0,1,0;
     -13.86*inertiacoeff,0,0;
     1,0,0];
 
 B = [0;inertiacoeff;0];
 
 Ref = -[0;0;4*pi/4];

eig(A)
 
Q = [1,0,0;
    0,16,0;
    0,0,96];
R = 1;

Kx = lqr(A,B,Q,R)

eig(A-B*Kx)

dt = .01;
t = 0:dt:10;
index=0;

Xrecord = zeros(3, length(t));
Xrecord(:,1) = X0;
X = X0;
U=0;
for time=t
    index = index+1;
    dX = A*X+B*U+Ref;
    X = X + dX*dt;
    U = -Kx(1:2)*X(1:2) - Kx(3)*X(3);
    %U = -Kx*X;
    Xrecord(:,index) = X;
    %hold on;
    plot(t(1:index), Xrecord(:,1:index));
    
    legend('Pos', 'Vel', 'S1');
    pause(.01);
end
%plot(t, Xrecord);
    
