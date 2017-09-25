%locomotion area
%takes speed and turn radius input (steering) into simulation and shows model
%TODO, add odometer module and take input from it (model that gives estimated distance travelled from wheel velocities)
%TODO, add path-turn compensation module and take input from it (turn towards position on path, uses odemter to determine what the position should be for the given path)
%TODO, add slip detection module and take input from it (throttle speed if wheel RPM is too high for input torque)

InitialRobotState = [1;0;0];
robotState = InitialRobotState;

waypoint = [3;1;pi/4]; % world coord
%transform waypoint to robot coordinates
%translate and then rotate
waypointTrans = waypoint - InitialRobotState;
waypointRot = [cos(InitialRobotState(3)), sin(InitialRobotState(3));-sin(InitialRobotState(3)),cos(InitialRobotState(3))] *  [waypointTrans(1);waypointTrans(2)];
waypointRobotCoord = [waypointRot(1);waypointRot(2);waypointTrans(3)]

[Distance, Radius, Distance2, Radius2, xcenterUT, ycenterUT, xcenter2UT, ycenter2UT] = waypoint2setpointsv3(waypointRobotCoord(1),waypointRobotCoord(2),waypointRobotCoord(3))
%transform turn centers to world coordinates
%translate, then rotate
center1 = [xcenterUT;ycenterUT] + [InitialRobotState(1);InitialRobotState(2)];
center1 =  [cos(InitialRobotState(3)), sin(InitialRobotState(3));-sin(InitialRobotState(3)),cos(InitialRobotState(3))] * [center1(1);center1(2)]

center2 = [xcenter2UT; ycenter2UT]+ [InitialRobotState(1);InitialRobotState(2)];
center2 =  [cos(InitialRobotState(3)), sin(InitialRobotState(3));-sin(InitialRobotState(3)),cos(InitialRobotState(3))] * [center2(1);center2(2)]

%Distance = 1;
%Radius = 2;%-.7;
%Distance2 = 1;
%Radius2 = -2;%.4;
firstwaypointmet =0;

dt = .01;
time = 0:dt:3;

wheelR = .3;
AxelLen = .5;

Ul =1;
Ur =1;

SpeedInput = 0;
SteeringInput = 0;
SteeringAccelInput=0;

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

%need to add path error/compensation term to control states
%how to do this?
%

 linearRobotWithServoStates = [0;Radius;0;0;0]; %start with the steering wheel in the right spot
 linearRobotWithServoA = [0,0,0,0,0; %integral of speedInput (distance)
                          0,0,0,0,.2; %integral of steeringAccel (Steering)
                         1,0,0,0,0; %error between distance and distance setpoint
                         0,1,0,0,0; %error between Steering and Steering setpoint
                         0,0,0,0,-.2] %path error/ turn compensation (truely: l1 norm between current pos and closest point on path, 
                                          %linearly: proportional to the difference between the current turn radius and the setpoint, but decreases with time?
                                          %could be steeringAccel - itself (if you are changing the wheel you are fucked up)
 linearRobotWithServoB = [1,0; %speed input
                          0,1; %steeringAccel
                          0,0; %no input to servo states
                          0,0; %no input to servo states
                          0,1];%if you are steering, you are compensating
 
linearRobotWithServoSetpoint = -[0;0;Distance;Radius;0];
 
 %naturalroots = eig(linearRobotWithServoA)
 
 linearRobotQ = [10,0,0,0,0;
                0,10,0,0,0;
                0,0,100,0,0;
                0,0,0,1000,0
                0,0,0,0,100];
  linearRobotR = [1,0;0,1];
  
  Kx = lqr(linearRobotWithServoA, linearRobotWithServoB, linearRobotQ, linearRobotR)
  
  ServoRoots = eig(linearRobotWithServoA - linearRobotWithServoB*Kx)

for t = time;

  [Ul, Ur] = sscv2(SpeedInput, SteeringInput, wheelR, AxelLen, 15); 
  %add noise to Ul Ur here
  
  
  drobotState = robotdynamics(Ul, Ur, robotState(3), dt, wheelR, AxelLen);
  robotState = robotState + drobotState;
  
  %read back estimate of control states from robot states
  %get speed from l2 norm of dx and dy
  speedEstimate = sqrt(drobotState(1)^2 + drobotState(2)^2);
  %get distance along arc from start to point on arc closest to current position? <---- winner
  %%this is also the point on the circle that is on the line through the center of the circle and the robots pos
  %%need to get center of circle from waypoint solver
  %%need to be careful about which solution is grabbed, dont want the other side of the circle
  %%this could all be done in world coordinates, which means waypointsolver center needs to be transformed through robots initial pose to the world
  
  %need to get path compensation/error term 
  %%desired x = r * cos (distance/r - pi/2)
  %%desired y = r * sin (distance/r - pi/2) + r
  %%  ^^ this is kind of the equation. actually need parameterized circle with distance=0 at robot's initial pose and center at the turn center
  %%%should be proportional to l1 norm of desired x,y and current x,y
  
  %should not need to estimate servo states?

   %switch setpoints if distance has been travelled
   if (linearRobotWithServoStates(1) > .95 * Distance && firstwaypointmet ==0) 
    linearRobotWithServoSetpoint = -[0;0;Distance2;Radius2;0];
    linearRobotWithServoStates = [0;Radius2;0;0;0];
    firstwaypointmet =1;
   end
   
   dlinearRobotWithServo = linearRobotWithServoA * linearRobotWithServoStates + linearRobotWithServoB * [SpeedInput;SteeringAccelInput] + linearRobotWithServoSetpoint;
   linearRobotWithServoStates = linearRobotWithServoStates + dlinearRobotWithServo*dt;
   
   input = -Kx(:,1:2)*linearRobotWithServoStates(1:2) - Kx(:,3:5)*linearRobotWithServoStates(3:5); %is the fifth state a servo state?
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
  %plot(center1(1),center1(2),'*');
  %plot(center2(1),center2(2),'*');
  %plot(waypoint(1),waypoint(2),'g*');
  
  
  
  hold off;

  %plot motor inputs
  subplot(2,2,2);
  Uplot(:,Uplotindex) = [Ul;Ur];
  plot([0:dt:t],Uplot(:,1:Uplotindex));
  legend('LeftWheel Speed', 'Right Wheel Speed');
  Uplotindex = Uplotindex+1;
  
  %plot speed and steering
  subplot(2,2,3);
  sscPlot(:,sscPlotindex) = [SpeedInput,SteeringAccelInput];
  plot([0:dt:t],sscPlot(:,1:sscPlotindex));
  legend('SpeedInput', 'SteeringAccelInput');
  sscPlotindex = sscPlotindex +1;
  
  %plot graph of servo states
  subplot(2,2,4);
  posPlot(:,posPlotindex) = [linearRobotWithServoStates(1:4)'];
  plot([0:dt:t], posPlot(1:2, 1:posPlotindex), '-', [0:dt:t], posPlot(3:4, 1:posPlotindex), '--');
  legend('Distance', 'TurnRadius', 'SpeedServo', 'RadiusServo');
  posPlotindex = posPlotindex+1; 
  
  
end
subplot(2,2,1);
robotdraw(robotState(1),robotState(2),robotState(3), linearRobotStateEstimate(1), linearRobotStateEstimate(2), linearRobotStateEstimate(3)); %needed to hold last frame