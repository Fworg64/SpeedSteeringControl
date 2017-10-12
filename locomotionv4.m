%%locomotionv4
%requirements
%0 steady state path error for any operable velocity of the robot (-.5/.5)
%m/s
%error may be generated with acceleration
%%so
%must track path error, its integral ,and derivitive
%%path error is signed distance of center of robot from closest path point

%%closest path point (CPP) is the closest point by perpindicular distance 
%%from the path to the center of the robot

%%must track theta error, the difference in the angle of the closest path
%%point and the robot (in world coordinates)
%%must also integrate and potentially derivate theta error (hopefully just
%%integrate)

%must estimate distance to goal for status purposes

%must accept unit step input with gain 1 (input should be speed setpoint)

%%linearstates
%left wheel speed = k1 * EPp + k2 * EPi + k3 * EPd + k4*ETp + k5*ETi + Ka*Ac - Ke*ELS
%right wheel speed = -(k1 * EPp + k2 * EPi + k3 * EPd + k4*ETp + k5*ETi) + Ka*Ac - Ke*ERS
%path error states 1-3 P I D
%EPp = EPkA * Ac + Decay, EPi = integral(EPp) and decay;
%EPd = 0 (get it from deriving linear state as sensor measurement and have
%it decay in this model)
%theta error states 1-2 P I
%ETp = ETkA * Ac + Decay, ETi = integral(ETp) and decay;
%general accel %grr, speed setpoint must operate accel
%Ac = AcKe*(speed error I) - (speed error D = Ac = Decay)
%let AcKe = .1
%left wheel speed error I
%right wheel speed error I


%%notes
%No input leads to diverging wheel steady state
%Input with LQR leads to steady state wheel vel below set point

wheeldecay = -.1;
k1 = .01; k2 = .02; k3 = -5; %EP  P I D
k4 = .01; k5 = .02;% ET P I
kA = .2; Ke = .4;
EPpDecay = -.1;
EPiDecay = -4;
EPkA = 1; ETkA = 1;
ETpDecay = -.01;
ETiDecay =-4;
AcDecay = -.4;
AcKe = -.1;
SpeedErrorDecay = -1;
EPdDecay = -.2;

numstates = 10
A = [wheeldecay, 0, k1, k2, k3, k4, k5, kA,-Ke, 0;%leftwheel (1)
     0,wheeldecay, -k1,-k2,-k3,-k4,-k5, kA, 0,-Ke;%rightwheel (2)
     0,0,     EPpDecay,  0,  0,  0,  0,EPkA,0,  0;%EPp (3)
     0,0,-1,   EPiDecay,  0,  0,  0,  0,  0,  0;%EPi (4)
     0,0,0,          0,  EPdDecay,  0,  0,  0,  0,   0;%EPd (5)
     0,0,0,          0,  0, ETpDecay, 0,ETkA,0,  0;%ETp (6)
     0,0,0,          0,  0,  1,  ETiDecay,  0,  0,  0;%ETi (7)
     0,0,0,          0,  0,  0, 0,AcDecay,AcKe,  AcKe;%(Ac)celleration (8)
     1,0,0,          0,  0,  0,      0, 0,  SpeedErrorDecay,  0;%left wheel speed error (9)
     0,1,0,          0,  0,    0,    0, 0,  0,  SpeedErrorDecay];%right wheel speed error (10)
     
Ref = zeros(10,1);
Ref (9) = .05; %m/s for the left wheel
Ref (10) = .05; %m/s for the right wheel

eig(A)

B = zeros(10,2);
B(1,1) = 1; B(2,1) = -1;
B(8,2) = 1;
%input affecting error derivitive
%B(5,1) = -.1;B(5,2) = .1;


Q = [1     0     0     0     0     0     0     0     0     0;
     0     1     0     0     0     0     0     0     0     0;
     0     0     10     0     0     0     0     0     0     0;
     0     0     0     10     0     0     0     0     0     0;
     0     0     0     0     10     0     0     0     0     0;
     0     0     0     0     0     10     0     0     0     0;
     0     0     0     0     0     0     10     0     0     0;
     0     0     0     0     0     0     0     1     0     0;
     0     0     0     0     0     0     0     0     10     0;
     0     0     0     0     0     0     0     0     0     10];
R = [1,0;
     0,10];
eig([Q,zeros(10,2);zeros(2,10),R]);
Kx = lqr(A,B,Q,R)
lqrPoles = eig(A - B*Kx)
X = zeros(10,1);
dt = .1;
time = 0:dt:60;

U = [0;0];

%robot simulation parts
robotPose = [1;0;pi/4]; %x, y, theta (world coord)
wheelR = .3;
AxelLen = .5;
LwheelSpeed = 0;
RwheelSpeed =0;
VescGains = [-8];

center = [1,1];
Radius = 1;

%measurement items
prevEPp =0;
prevLwheelSpeed=0;
prevRwheelSpeed=0;

     EPpEst = 0; %positive error means turn right, assume robot is pointing the correct direction
     EPiEst = 0;     EPdEst = 0;
     prevEPp = 0;     ETpEst = 0;
     ETiEst = 0;     AcEst = 0;
     prevLwheelSpeed = 0;
     LWSEEst = 0;     RWSEEst = 0;

%observer gains
ObserverGains = [0,0, .1,.1,.1, .1,.1, 0,0,0];

Xplot = zeros(length(X),length(time));
Uplot = zeros(length(U),length(time));
Wheelplot = zeros(2, length(time));
Estplot = zeros(length(X), length(time));

XplotIndex =1;

plotperiod = 1;
plotindex=1;
figure();
for t = time;
     %do physical sim
     LwheelSpeed = LwheelSpeed + VescGains(1)*(LwheelSpeed - X(1))*dt;
     RwheelSpeed = RwheelSpeed + VescGains(1)*(RwheelSpeed - X(2))*dt;
     dRobot = robotdynamics(LwheelSpeed,RwheelSpeed, robotPose(3), dt, wheelR, AxelLen);
     robotPose = robotPose + dRobot;
     
     measuredRobotPose = robotPose + .001*rand(3,1);
     
     %get sensor measurements
     [CPPx,CPPy,CPPth] = findCPP(measuredRobotPose(1), measuredRobotPose(2), center(1), center(2), Radius);
     EPpEst = Radius - sqrt((center(1) - measuredRobotPose(1))^2 + (center(2) - measuredRobotPose(2))^2); %positive error means turn right, assume robot is pointing the correct direction
     EPiEst = EPiEst + EPpEst*dt;
     EPdEst = (X(3) - prevEPp)*dt;
     prevEPp = X(3);
     ETpEst = calculateDifferenceBetweenAngles(measuredRobotPose(3),CPPth); %be careful with difference in angles here pi=0;
     ETiEst =ETiEst +ETpEst*dt;
     AcEst = ((LwheelSpeed + RwheelSpeed) - (prevLwheelSpeed +prevRwheelSpeed))*dt;
     prevLwheelSpeed = LwheelSpeed;prevRwheelSpeed = RwheelSpeed;
     LWSEEst = LwheelSpeed - Ref(9);
     RWSEEst = RwheelSpeed - Ref(10);
     
     %get residual between state outputs and sensor measurements
     
     Res = X - [LwheelSpeed;RwheelSpeed;EPpEst;EPiEst;EPdEst;ETpEst;ETiEst;AcEst;LWSEEst;RWSEEst];
     
     %do control system
     dX = A*X - Ref +B*U +ObserverGains*Res;
     X = X + dX*dt;
     %U =[0;0];
     U = -Kx*X;
     
     
     
     %plug wheel velocity commands into vesc
     %simulate robot
     %draw robots path through time?
     
     Xplot(:,XplotIndex) = X;
     Uplot(:,XplotIndex) = U;
     Wheelplot(1,XplotIndex) = LwheelSpeed;
     Wheelplot(2,XplotIndex) = RwheelSpeed;
     Estplot(:,XplotIndex) = [LwheelSpeed;RwheelSpeed;EPpEst;EPiEst;EPdEst;ETpEst;ETiEst;AcEst;LWSEEst;RWSEEst];
     if (t> plotperiod * plotindex)
         plotindex = plotindex+1;
              subplot(2,3,1);
         plot([0:dt:t], Xplot(1:2,1:XplotIndex), [0:dt:t],Uplot(:,1:XplotIndex), '--',[0:dt:t],Wheelplot(:,1:XplotIndex),'o');
         legend('Left Wheel Speed','Right Wheel Speed','Speed Input', 'Steering Input' ,'LeftVesc','RightVesc');
              subplot(2,3,2);
         plot([0:dt:t], Xplot(3:5,1:XplotIndex),[0:dt:t],Estplot(3:5,1:XplotIndex),'--');
         legend('EPp', 'EPi', 'EPd', 'EPpEst', 'EPiEst', 'EPdEst');
              subplot(2,3,3);
         plot([0:dt:t], Xplot(6:7,1:XplotIndex),[0:dt:t],Estplot(6:7,1:XplotIndex),'--');
         legend('ETp', 'ETi', 'ETpEst', 'ETiEst');
              subplot(2,3,4);
         plot([0:dt:t], Xplot(8:10,1:XplotIndex), [0:dt:t],Estplot(8:10,1:XplotIndex),'--');
         legend('Accel', 'Left Wheel Servo', 'Right Wheel Servo', 'Accel Est', 'Left Wheel Servo Est', 'Right Wheel Servo Est');
              subplot(2,3,5);
         hold on;
         robotdraw(robotPose(1),robotPose(2),robotPose(3),0,0,0);
         hold off;
         
         pause(.01);
     end
     XplotIndex = XplotIndex +1;
end
     
