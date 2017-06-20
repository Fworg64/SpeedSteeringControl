%lqr and MIMO servo review

A6 = [0,0,1,0,0,0;
      0,0,0,1,0,0;
      0,0,0,0,1,0;
      0,0,0,0,0,1;
      0,0,0,0,0,0;
      0,0,0,0,0,0];
B6 = [0,0;
      0,0;
      0,0;
      0,0;
      1,0;
      0,1];
C6 = [1,0,0,0,0,0;
      0,1,0,0,0,0;
      0,0,1,0,0,0;
      0,0,0,1,0,0];
Q = C6'*C6;
R = [100,0;
     0,1];
Kx = lqr(A6,B6,Q,R)
eig(A6- B6*Kx)

%add 2 state servo for input 1

Az = [-1,0;0,-1];
Bz = [1;1];
Cz = [1,0,0,0,0,0];

A8 = [A6, zeros(6,2);Bz*Cz, Az];
B8 = [B6;zeros(2,2)];
C8 = [C6,zeros(4,2)];
%Q8 = C8'*C8;
Q8 = zeros(8,8) + 1e-4*eye(8);
Q8(8,8) = 1000;
Q8(7,7) = 1000;
Q8(1,1) = 100;
Kx2 = lqr(A8, B8, Q8, R)
eig(A8 - B8*Kx2)

Cz2 = [0,1,0,0,0,0,0,0];
A10 = [A8, zeros(8,2);Bz*Cz2, Az];
B10 = [B8;zeros(2,2)];
C10 = [C8, zeros(4,2)];
Q10 = zeros(10,10) + 1e-5*eye(10);
for index = 7:10
    Q10(index,index) = 1000;
end
Kx3 = lqr(A10, B10, Q10, R)
eig(A10 - B10*Kx3)

dt = .01;
t=0:dt:10;
Xrecord = zeros(10,length(t));
X = zeros(10,1);
XrecordIndex =1;

for time=t
    U = -Kx3(:,1:6)*X(1:6) - Kx3(:,7:10)*X(7:10);
    dX = A10 * X + B10*U - [1.5;1;0;0;0;0;0;0;0;0];
    X = X + dX*dt;
    Xrecord(:,XrecordIndex) = X;
    subplot(3,1,1);
    plot([0:dt:time], Xrecord(1:4,1:XrecordIndex));
    legend('distance', 'NAD', 'Speed', 'Steer');
    subplot(3,1,2);
    plot([0:dt:time], Xrecord(5:6,1:XrecordIndex));
    legend('Accel', 'Steer vel');
    subplot(3,1,3);
    plot([0:dt:time], Xrecord(7:10,1:XrecordIndex));
    legend('Servo1_1', 'Servo1_2', 'Servo2_1', 'Servo2_2');
    XrecordIndex = XrecordIndex +1;
    pause(.01);
end
    