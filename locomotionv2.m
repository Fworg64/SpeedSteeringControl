%locomotion area
%takes speed and turn radius input (steering) into simulation and shows model

waypoint = [2,1,pi/4]; %relative to robot
%[Distance, Radius, Distance2, Radius2] = waypoint2setpointsv3(waypoint(1),waypoint(2),waypoint(3))

Distance = 1;
Radius = 5;%-.7;
Distance2 = 1;
Radius2 = 5;%.4;

dt = .01;
time = 0:dt:2;

wheelR = .3;
AxelLen = .5;

Ul =1;
Ur =1;

SpeedInput = 0;
SteeringInput = 0;
SteeringAccelInput=0;

InitialRobotState = [1;1;0];
robotState = InitialRobotState;

%plotting records
crumbcounter = 21;
crumbperiod =20;
crumbindex=0;
crumbs = zeros(2,int32(length(time)/crumbperiod));

Uplot = zeros(2,length(time));
Uplotindex=1;

sscPlot = zeros(2,length(time));
sscPlotindex=1;

posPlot = zeros(4, length(time));
posPlotindex=1;

linearRobotStateEstimate = [0;0;0];
% 
% linearRobotStateEstimate = [robotState;0];
% linearRobotModelA = [0,0,0,-.1;
%                      0,0,0,0;
%                      0,0,0,0;
%                      0,0,1,0];
% linearRobotB = [2,-1;0,-.5;0,-.10;0,0]; %linear model from velocitiy inputs, includes integral of 

% 
%linearRobotStateEstimate = [robotState;0;0];
%linearRobotModelA = [0,0,0,1,0;%dx pos
%                    0,0,-.1,0,0;%dy pos
%                    0,0,0,0,-2;%dtheta
%                    0,0,0,0,0; %integral of Speed
%                    0,0,0,0,0];%integral of Steering
%linearRobotB = [1,0;
%                0,-2;
%                0,-3;
%                1,0;
%                0,1]; %Speed and steering input

 linearRobotWithServoStates = [0;Radius;0;0]; %start with the steering wheel in the right spot
 linearRobotWithServoA = [0,0,0,0; %integral of speedInput (distance)
                          0,0,0,0; %integral of steeringAccel (Steering)
                         1,0,0,0; %error between distance and distance setpoint
                         0,1,0,0];%error between Steering and Steering setpoint
 linearRobotWithServoB = [1,0;0,1;0,0;0,0];
 
linearRobotWithServoSetpoint = -[0;0;Distance;Radius];
 
 %naturalroots = eig(linearRobotWithServoA)
 
 linearRobotQ = [10,0,0,0;
                0,10,0,0,;
                0,0,100,0;
                0,0,0,100000];
  linearRobotR = [1,0;0,1];
  
  Kx = lqr(linearRobotWithServoA, linearRobotWithServoB, linearRobotQ, linearRobotR)
  
  ServoRoots = eig(linearRobotWithServoA - linearRobotWithServoB*Kx)

for t = time;

  %[Ul, Ur] = ssc(SpeedInput, SteeringInput, Ul, Ur);
  [Ul, Ur] = sscv2(SpeedInput, SteeringInput, wheelR, AxelLen, 15);
  
  drobotState = robotdynamics(Ul, Ur, robotState(3), dt, wheelR, AxelLen);
  robotState = robotState + drobotState;
    
   %dlinearRobot = linearRobotModelA * linearRobotStateEstimate + linearRobotB*[SpeedInput;SteeringInput];
   %linearRobotStateEstimate = linearRobotStateEstimate + dlinearRobot*dt;
   
   if (linearRobotWithServoStates(1) > .95 * Distance) %switch setpoints if distance has been travelled
    linearRobotWithServoSetpoint = -[0;0;Distance2;Radius2];
    linearRobotWithServoStates = [0;0;0;0];
   end
   
   dlinearRobotWithServo = linearRobotWithServoA * linearRobotWithServoStates + linearRobotWithServoB * [SpeedInput;SteeringAccelInput] + linearRobotWithServoSetpoint;
   linearRobotWithServoStates = linearRobotWithServoStates + dlinearRobotWithServo*dt;
   
   input = -Kx(:,1:2)*linearRobotWithServoStates(1:2) - Kx(:,3:4)*linearRobotWithServoStates(3:4);
  SpeedInput = input(1);
  SteeringAccelInput = input(2);
  SteeringInput = linearRobotWithServoStates(2);
    
  %plot robot
  subplot(2,2,1);
  hold on;
  robotdraw(robotState(1),robotState(2),robotState(3),linearRobotStateEstimate(1), linearRobotStateEstimate(2), linearRobotStateEstimate(3));
  %%plot path points
  %scatter(path(1,:),path(2,:),'green','*');
  %record crumbs
  crumbcounter = crumbcounter+1;
  if (crumbcounter>crumbperiod)
    crumbcounter=0;
    crumbindex = crumbindex+1;
    crumbs(:,crumbindex) = robotState(1:2);
  end
  scatter(crumbs(1,1:crumbindex),crumbs(2,1:crumbindex));
  %plot waypoint
  hold off;

  %plot motor inputs
  subplot(2,2,2);
  Uplot(:,Uplotindex) = [Ul;Ur];
  plot([0:dt:t],Uplot(:,1:Uplotindex));
  legend('LeftWheel Speed', 'Right Wheel Speed');
  Uplotindex = Uplotindex+1;
  
  %plot speed and steering
  subplot(2,2,3);
  sscPlot(:,sscPlotindex) = [SpeedInput,SteeringInput];
  plot([0:dt:t],sscPlot(:,1:sscPlotindex));
  legend('SpeedInput', 'SteeringInput');
  sscPlotindex = sscPlotindex +1;
  
  %plot graph of servo states
  subplot(2,2,4);
  posPlot(:,posPlotindex) = [linearRobotWithServoStates(1:4)'];
  plot([0:dt:t], posPlot(1:2, 1:posPlotindex), '-', [0:dt:t], posPlot(3:4, 1:posPlotindex), '--');
  legend('Distance', 'SteeringInput', 'SpeedServo', 'SteeringServo');
  posPlotindex = posPlotindex+1; 
  
  
end
subplot(2,2,1);
robotdraw(robotState(1),robotState(2),robotState(3), linearRobotStateEstimate(1), linearRobotStateEstimate(2), linearRobotStateEstimate(3)); %needed to hold last frame