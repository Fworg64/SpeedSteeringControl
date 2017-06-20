%path follower, locomotion

%path is defined as collection of poses consisting of 
%         position and angle (x,y,theta)

%x is forward, y is left, theta is clockwise

path = [.75,-1,0;
        2.5,0,20*pi/180;
        5,0,0]'
        

dt = .01;
t = 0:dt:20;

%X = path(:,1); %x,y,theta
roboX = [1;1;0];
clf
crumbcounter =21;
crumbperiod = 20;
crumbindex=0;
crumbs = zeros(2,int32(length(t)/crumbperiod));

Uplot = zeros(2,length(t));
Uplotindex=1;

sscPlot = zeros(12,length(t));
sscPlotindex=1;

Ul=1;
Ur=0;

Cz1 = [0,0,1,0,0,0]; Bz1 = [1;1;1]; Az1 = [-10,0,0;0,-30,3;0,-3,-30];
Cz2 = [0,0,0,1,0,0]; Bz2 = [1;1;1]; Az2 = [-10,0,0;0,-30,3;0,-3,-30];


%A = [0,0,1,0,0,0,0,0,0,0,0,0; %distance
%     0,0,0,1,0,0,0,0,0,0,0,0; %net angular distance (NADs)
%     0,0,0,0,1,0,0,0,0,0,0,0; %speed
%     0,0,0,0,0,1,0,0,0,0,0,0; %steering
%     0,0,0,0,0,0,0,0,0,0,0,0; %accel
%     0,0,0,0,0,0,0,0,0,0,0,0; %steering vel (how fast to turn the wheel)
%     0,0,1,0,0,0,-1,0,0,0,0,0;%speed servo
%     0,0,1,0,0,0,0,-3,.3,0,0,0;%speed servo
%     0,0,1,0,0,0,0,-.3,-3,0,0,0;%speed servo
%     0,0,0,1,0,0,0,0,0,-1,0,0;%steering servo
%     0,0,0,1,0,0,0,0,0,0,-3,.3;%steering servo
%     0,0,0,1,0,0,0,0,0,0,-.3,-3];%steering servo
A6 = [0,0,1,0,0,0;%distance
      0,0,0,1,0,0;%NAD
      0,0,0,0,1,0;%speed
      0,0,0,0,0,1;%steering
      0,0,0,0,0,0;%accel
      0,0,0,0,0,0];%steering vel (how fast to turn the wheel)
A = [A6, zeros(6,6);Bz1*Cz1, Az1, zeros(3,3);Bz2*Cz2,zeros(3,3),Az2];

B = [0,0;
     0,0;
     0,0;
     0,0;
     1,0;
     0,1;
     0,0;
     0,0;
     0,0;
     0,0;
     0,0;
     0,0];
C = [1,0,0,0,0,0,0,0,0,0,0,0;
     0,1,0,0,0,0,0,0,0,0,0,0
     0,0,1,0,0,0,0,0,0,0,0,0;
     0,0,0,1,0,0,0,0,0,0,0,0;];
%Q = C'*C;
%Q = diag([0,0,1,1,0,0,0,0,0,0,0,0]) + 1e-6;
Q = zeros(12,12) + 1e-5*eye(12);
for index = 7:12
    Q(index,index) = 100; %weight servo states
end
Q(5,5) = 10;
Q(6,6) = 10;
R = [100,0; %want to adjust steering more
     0,10]; % so weight speed cost higher
Kx = lqr(A,B,Q,R) %need lqr for MIMO
eig(A- B*Kx) %check poles of system

stateX = zeros(12,1) ;
    %distance traveled
    %angle traveled
    %speed
    %steering
    %accel
    %steering vel
    %steering servo state
    %speed servo state
sscInput = zeros(1,2);

for test = t

  SpeedSetPoint = 1;
  SteeringSetPoint =.5;
    
  sscInput = -Kx(:,1:6)*stateX(1:6) - Kx(:,7:12) * stateX(7:12);
  [Ul, Ur] = ssc(stateX(3), stateX(4), Ul, Ur);
  droboX = robotdynamics(Ul, Ur, roboX(3), dt);
  roboX = roboX + droboX;
  dstateX = A(1:6,1:6)*stateX(1:6) + B(1:6,:)*sscInput;
  stateX(1:6) = stateX(1:6) + dstateX*dt;
  dstateZ1 = Az1 * stateX(7:9) + Bz1*(stateX(3) - SpeedSetPoint); %implicit Cz1
  dstateZ2 = Az2 * stateX(10:12) + Bz2*(stateX(4) - SteeringSetPoint); %implicit Cz1
  stateX(7:9) = stateX(7:9) + dstateZ1*dt;
  stateX(10:12) = stateX(10:12) + dstateZ2*dt;
  sscControlY = eye(12) * stateX;
  
  
  %plot robot
  subplot(3,1,1);
  hold on;
  robotdraw(roboX(1),roboX(2),roboX(3));
  %plot path points
  scatter(path(1,:),path(2,:),'green','*');
  %record crumbs
  crumbcounter = crumbcounter+1;
  if (crumbcounter>crumbperiod)
    crumbcounter=0;
    crumbindex = crumbindex+1;
    crumbs(:,crumbindex) = roboX(1:2);
  end
  scatter(crumbs(1,1:crumbindex),crumbs(2,1:crumbindex));

  %plot motor inputs
  subplot(3,1,2);
  Uplot(:,Uplotindex) = [Ul;Ur];
  plot([0:dt:test],Uplot(:,1:Uplotindex));
  Uplotindex = Uplotindex+1;
  
  %plot sscSystem outputs
  subplot(3,1,3);
  sscPlot(:,sscPlotindex) = sscControlY;
  plot([0:dt:test],sscPlot(:,1:sscPlotindex));
  sscPlotindex = sscPlotindex +1;
end
subplot(3,1,3);
legend('distance', 'angle travel', 'speed', 'steering','accel', 'steering vel', 'Steer\_servo', 'Speed\_servo');
%legend('speed', 'steering');
subplot(3,1,1);
  robotdraw(roboX(1),roboX(2),roboX(3)); %needed to hold last frame



