%locomotion area
%takes speed and steering input into simulation and shows model

%simulation variables
dt = .01;
time = 0:dt:2;

Ul =1;
Ur =1;

SpeedInput = 4;
SteeringInput = 3.5;

robotState = [1;1;0];

%plotting records
crumbcounter = 21;
crumbperiod =20;
crumbindex=0;
crumbs = zeros(2,int32(length(time)/crumbperiod));

Uplot = zeros(2,length(time));
Uplotindex=1;

sscPlot = zeros(2,length(time));
sscPlotindex=1;
% 
% linearRobotStateEstimate = [robotState;0];
% linearRobotModelA = [0,0,0,-.1;
%                      0,0,0,0;
%                      0,0,0,0;
%                      0,0,1,0];
% linearRobotB = [2,-1;0,-.5;0,-.10;0,0]; %linear model from velocitiy inputs, includes integral of 

% 
linearRobotStateEstimate = [robotState;0;0];
linearRobotModelA = [0,0,-1,3,0;
                    0,0,1,0,-5;
                    0,0,0,0,-10;
                    0,0,0,0,0; %integral of Speed
                    0,0,0,0,0];%integral of Steering
linearRobotB = [0,0;0,0;0,0;1,0;0,1]; %Speed and steering input

 linearRobotWithServoStates = [robotState;0;0;0;0];
 linearRobotWithServoA = [linearRobotModelA, zeros(5, 2);
                         1,0,0,0,0,0,0;
                         0,1,0,0,0,0,0];
 linearRobotWithServoB = [linearRobotB;zeros(2,2)];
 
 linearRobotWithServoSetpoint = [0;0;0;0;0;-2;-2];
 
 naturalroots = eig(linearRobotWithServoA)
 
 linearRobotQ = [1,0,0,0,0,0,0;
      0,1,0,0,0,0,0;
      0,0,1,0,0,0,0;
      0,0,0,1,0,0,0;
      0,0,0,0,1,0,0;
      0,0,0,0,0,10,0;
      0,0,0,0,0,0,10];
  linearRobotR = [1,0;0,5];
  
  Kx = lqr(linearRobotWithServoA, linearRobotWithServoB, linearRobotQ, linearRobotR)
  
  ServoRoots = eig(linearRobotWithServoA - linearRobotWithServoB*Kx)

for t = time;

  [Ul, Ur] = ssc(SpeedInput, SteeringInput, Ul, Ur);
  
  drobotState = robotdynamics(Ul, Ur, robotState(3), dt);
  robotState = robotState + drobotState;
    
   dlinearRobot = linearRobotModelA * linearRobotStateEstimate + linearRobotB*[SpeedInput;SteeringInput];
   linearRobotStateEstimate = linearRobotStateEstimate + dlinearRobot*dt;
   
   dlinearRobotWithServo = linearRobotWithServoA * linearRobotWithServoStates + linearRobotWithServoB * [SpeedInput;SteeringInput] + linearRobotWithServoSetpoint;
   linerRobotWithServoStates = linearRobotWithServoStates + dlinearRobotWithServo*dt;
   
   %input = -Kx(:,1:5)*linearRobotWithServoStates(1:5) - Kx(:,6:7)*linearRobotWithServoStates(6:7);
   %SpeedInput = input(1);
   %SteeringInput = input(2);
    
  %plot robot
  subplot(3,1,1);
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
  hold off;

  %plot motor inputs
  subplot(3,1,2);
  Uplot(:,Uplotindex) = [Ul;Ur];
  plot([0:dt:t],Uplot(:,1:Uplotindex));
  Uplotindex = Uplotindex+1;
  
  %plot speed and steering
  subplot(3,1,3);
  sscPlot(:,sscPlotindex) = [SpeedInput,SteeringInput];
  plot([0:dt:t],sscPlot(:,1:sscPlotindex));
  sscPlotindex = sscPlotindex +1;
  
end
subplot(3,1,1);
robotdraw(robotState(1),robotState(2),robotState(3)); %needed to hold last frame