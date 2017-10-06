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
time = 0:dt:1;

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

posPlot = zeros(5, length(time));
posPlotindex=1;

%vecotr of measured states
measuredStates = zeros(3,length(time));
mStatesPlotIndex = 1;
%estimate of robot states from linear model
linearRobotStateEstimate = [0;0;0];

TurnCompensatorEffect = 2;
TurnCompensatorObservationGain = 6;
TurnCompensatorInputGain = .2;
TurnCompensatorEstimateDecay = -.6;
turnCompensationMeasurementScale = 2;
% a brief note on radius vs steering. Let the turn radius be the inverse of 
%steering. So Radius = 1/Steering. So as Sterring -> 0, Radius -> +/- Inf

 linearRobotWithServoStates = [0;Radius;0;0;0]; %start with the Turn Radius in the right spot
 linearRobotWithServoA = [0,0,0,0,0; %integral of speedInput (distance)
                          0,0,0,0,-TurnCompensatorEffect; %integral of TurnRadiusAccel (turn radius), to use compensation, 
                         1,0,0,0,0; %error between distance and distance setpoint
                         0,1,0,0,0; %error between TurnRadius and TurnRadius setpoint
                         0,0,0,0,TurnCompensatorEstimateDecay] %path error/ turn compensation 
                                          %truely:proportional to signed l1 norm
                                          %between current pos and closest point
                                          %on path, measured in Steering
                                          %linear estimate: weakly proportional 
                                          %to the difference between the current
                                          %turn radius and the setRadius, and
                                          % decreases with time?
                                          %
%strategy, inject turn compensation after speed steering/radius control
% or, make speed steering/radius control work on its own
 linearRobotWithServoB = [1,0; %speed input
                          0,1; %steeringAccel
                          0,0; %no input to servo states
                          0,0; %no input to servo states
                          0,TurnCompensatorInputGain];%if you are doing steeringAccel, you are compensating
 
linearRobotWithServoSetpoint = -[0;0;Distance;Radius;0];
 
 %naturalroots = eig(linearRobotWithServoA)
 
 linearRobotQ = [10,0,0,0,0;
                0,10,0,0,0;
                0,0,100,0,0;
                0,0,0,10,0
                0,0,0,0,10000];
  linearRobotR = [1,0;0,1];
  
  Kx = lqr(linearRobotWithServoA, linearRobotWithServoB, linearRobotQ, linearRobotR)
  
  ServoRoots = eig(linearRobotWithServoA - linearRobotWithServoB*Kx)
  
  %pick observer gains, noise not yet characterized, so it is done by hand
  %should use pole placement, investigate LQR methods
  %observerinput = eye(5);
  %observerinput(3:4,3:4) = 0;
  %ObserverGains = ppl(linearRobotWithServoA',observerinput', [-4,-3,-2,-1,-5])
  ObserverGains = [2,2,0,0,TurnCompensatorObservationGain]; %how much to weight the difference between estimated states and measured states
  %to make it a kalman-bucy filter, use lqr methods to make them proportional to the noise of each sensor
  Residual = [0;0;0;0;0]; %the unweighted difference between the estimated states and the measured states
  
  %ObserverRoots = eig(linearRobotWithServoA' -eye(5)'*ObserverGains')

for t = time;

  [Ul, Ur] = sscv2(SpeedInput, SteeringInput, wheelR, AxelLen, 15); 
  %add noise to Ul Ur here
  
  
  drobotState = robotdynamics(Ul, Ur, robotState(3), dt, wheelR, AxelLen);
  robotState = robotState + drobotState;
  
  %read back estimate of control states from robot states
  %get speed from l2 norm of dx and dy
  %speedEstimate = sqrt(drobotState(1)^2 + drobotState(2)^2); %do we actually need this?
  %need turn radius estimate
  %rotational velocity = (rightwheelspeed - leftwheelspeed)/AxelLen
  %turn radius:  R = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
  if (Ur - Ul ==0)
    RadiusEstimate = 1;%100;
  else
    RadiusEstimate = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
  end
  
  %get distance along arc from start to point on arc closest to current position? <---- winner
  %%this is also the point on the circlur path that is on the line through the center of the circle and the robots pos
  %%need to get center of circle from waypoint solver - got it
  %%care! need to be careful about which solution is grabbed, dont want the other side of the circle
  %%this could all be done in world coordinates, which means waypointsolver center needs to be transformed through robots initial pose to the world
  %thisLineEq: y - center1(2) = (center1(2) - robotState(2))/(center1(1) - robotState(1)) * (x - center1(1));
  %pathEq: Radius^2 = (x - center1(1))^2 + (y - center1(2))^2;
  %will be the solution that is closest to robot pose - this is the care!

  %need to get path compensation/error term 
  %%desired x = r * cos (distance/r - pi/2)
  %%desired y = r * sin (distance/r - pi/2) + r
  %%  ^^ this is kind of the equation. actually need parameterized circle with distance=0 at robot's initial pose and center at the turn center
  %%%should be proportional to l1 norm of desired x,y and current x,y
  %%%also would be distance between closest path point and robot pose, as calculated above
  
  
  
  %get path compensation term first by finding nearest point on path and taking signed difference between robot pose and closest path point
  %%nearest point on path is point on path that is on the line through the center of the circle and the robots pose that is closest to the robots pose.
  %%%calculating path compensation term
  %%care! need to be careful about which solution is grabbed, dont want the other side of the circle
  %%this could all be done in world coordinates, which means waypointsolver center needs to be transformed through robots initial pose to the world
  %thisLineEq: y - center1(2) = (center1(2) - robotState(2))/(center1(1) - robotState(1)) * (x - center1(1));
  %pathEq: Radius^2 = (x - center1(1))^2 + (y - center1(2))^2;
  %will be the solution that is closest to robot pose - this is the care!
  if (firstwaypointmet ==0)
    if (center1(1) - robotState(1) ==0)
      M = (center1(2) - robotState(2))/.001;
    else
      M = (center1(2) - robotState(2))/(center1(1) - robotState(1));
    end
    potXpa = (-(-2*center1(1) - 2*M^2*center1(1)) + sqrt((-2*center1(1) - 2*M^2*center1(1))^2 - 4*(1+M^2)*(center1(1)^2 - Radius^2 + M^2*center1(1)^2)))/(2*(1 + M^2));
    potXpb = (-(-2*center1(1) - 2*M^2*center1(1)) - sqrt((-2*center1(1) - 2*M^2*center1(1))^2 - 4*(1+M^2)*(center1(1)^2 - Radius^2 + M^2*center1(1)^2)))/(2*(1 + M^2));
    if (abs(robotState(1) - potXpa) < abs(robotState(1) - potXpb))
     Xp = potXpa;
    else
     Xp = potXpb;
    end
    Yp = M*(Xp - center1(1)) + center1(2);

  else
    if (center2(1) - robotState(1) ==0)
     M = (center2(2) - robotState(2))/.001;
    else
     M = (center2(2) - robotState(2))/(center2(1) - robotState(1));
    end
    potXpa = (-(-2*center2(1) - 2*M^2*center2(1)) + sqrt((-2*center2(1) - 2*M^2*center2(1))^2 - 4*(1+M^2)*(center2(1)^2 - Radius^2 + M^2*center2(1)^2)))/(2*(1 + M^2));
    potXpb = (-(-2*center2(1) - 2*M^2*center2(1)) - sqrt((-2*center2(1) - 2*M^2*center2(1))^2 - 4*(1+M^2)*(center2(1)^2 - Radius^2 + M^2*center2(1)^2)))/(2*(1 + M^2));
    if (abs(robotState(1) - potXpa) < abs(robotState(1) - potXpb))
     Xp = potXpa;
    else
     Xp = potXpb;
    end
    Yp = M*(Xp - center2(1)) + center2(2);

  end
  %pathCompensationEstimate = (robotState(1) - Xp) + (robotState(2) - Yp); %probably, might want a log or something. need to be careful with oscilliations
  %pathCompensationEstimate = robotState(2) - ((-1/M) * (robotState(1) - Xp) + Yp); %robot's height above or below the instantaneous line of the path at Xp,Yp
  %still probably not good enough, need something clearly defined for any slope. Like maybe distance inside or outside the radius of the turn?
  %what about robots distance from the center - path's distance from the center? -< yess
  
  %reminder that this is being measured in steering...
  %this number will be negative if a larger turn radius is needed
  %and positive if a smaller turn radius is needed. So subtract it.
  
  %must be "faster" then the servo trying to "fix" the turn radius at the set point
  if (firstwaypointmet ==0)
    pathCompensationEstimate = sqrt((center1(1) - robotState(1))^2 + (center1(2) - robotState(2))^2) - sqrt((center1(1) - Xp)^2 + (center1(2) - Yp)^2);
  else
    pathCompensationEstimate = sqrt((center2(1) - robotState(1))^2 + (center2(2) - robotState(2))^2) - sqrt((center2(1) - Xp)^2 + (center2(2) - Yp)^2);
  end
  pathCompensationEstimate = turnCompensationMeasurementScale*pathCompensationEstimate;
  %get distance traveled from closest path point and back calculating distance travelled along arc
  %Xp = r * cos(distance/r - pi/2)
  %Yp = r * sin(distance/r - pi/2) +r
    %%  ^^ this is kind of the equation. actually need parameterized circle with distance=0 at robot's initial pose and center at the turn center
    %% i.e. it needs transformed if robot has turned first. Might be able to get some special case magic...
    
  %acos gives 0 to pi for 1 to -1
  %asin gives pi/2 to -pi/2 for 1 to -1
  %distance is arcangle * r, r is given
  %arcangle is angle of closest point on circle - angle of startpoint on circle
  %angle of startpoint on circle is -pi/2 for first jaunt and distance1/Radius1 - pi/2 for second jaunt
  if (firstwaypointmet ==0)
    %%need to be careful about which quadrant life is in, can test with xp and yp relative to xc and yc
    %%only really need to know top half or bottom half
    arcangle = (acos((Xp - center1(1))/Radius) - pi/2);
    if (Yp > center1(2))
      arcangle = arcangle + pi;
    end
    distanceTravelledEstimate = -Radius * arcangle;
  else
    arcangle = (asin((Xp - center2(1))/Radius2) - (Distance/Radius - pi/2));
    if (Yp > center2(2))
      arcangle = arcangle + pi;
    end
    distanceTravelledEstimate = -Radius2 * arcangle;
  end
  
  
  
  %should not need to estimate servo states?

   %switch setpoints if distance has been travelled
   %if (linearRobotWithServoStates(1) > .95 * Distance && firstwaypointmet ==0) 
   % linearRobotWithServoSetpoint = -[0;0;Distance2;Radius2;0];
   % linearRobotWithServoStates = [0;Radius2;0;0;0];
   % firstwaypointmet =1;
   %end
   
   %get residual
   Residual = [distanceTravelledEstimate; RadiusEstimate; 0;0; pathCompensationEstimate] - linearRobotWithServoStates;
   
   dlinearRobotWithServo = linearRobotWithServoA * linearRobotWithServoStates + linearRobotWithServoB * [SpeedInput;SteeringAccelInput] + ObserverGains * Residual +linearRobotWithServoSetpoint;
   linearRobotWithServoStates = linearRobotWithServoStates + dlinearRobotWithServo*dt;
   
   %input = -Kx(:,1:2)*linearRobotWithServoStates(1:2) - Kx(:,3:5)*linearRobotWithServoStates(3:5); %is the fifth state a servo state?
   input = -Kx(:,1:2)*linearRobotWithServoStates(1:2) - Kx(:,3:4)*linearRobotWithServoStates(3:4) - Kx(:,5) * linearRobotWithServoStates(5);
  SpeedInput = input(1);
  SteeringAccelInput = input(2);
  SteeringInput = linearRobotWithServoStates(2);
    
  %plot robot
  subplot(3,2,1);
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
  subplot(3,2,2);
  Uplot(:,Uplotindex) = [Ul;Ur];
  plot([0:dt:t],Uplot(:,1:Uplotindex));
  legend('LeftWheel Speed', 'Right Wheel Speed');
  Uplotindex = Uplotindex+1;
  
  %plot speed and steering
  subplot(3,2,3);
  sscPlot(:,sscPlotindex) = [SpeedInput,SteeringAccelInput];
  plot([0:dt:t],sscPlot(:,1:sscPlotindex));
  legend('SpeedInput', 'SteeringAccelInput');
  sscPlotindex = sscPlotindex +1;
  
  %plot graph of servo states
  subplot(3,2,4);
  posPlot(:,posPlotindex) = [linearRobotWithServoStates(1:5)'];
  plot([0:dt:t], posPlot(1:2, 1:posPlotindex), '-', [0:dt:t], posPlot(3:4, 1:posPlotindex), '--', [0:dt:t], posPlot(5,1:posPlotindex), '*');
  legend('Distance', 'TurnRadius', 'SpeedServo', 'RadiusServo', 'TurnCompensator');
  posPlotindex = posPlotindex+1; 
  
  %plot graph of estimated states
  subplot(3,2,5);
  measuredStates(:,mStatesPlotIndex) = [distanceTravelledEstimate, RadiusEstimate, pathCompensationEstimate];
  plot([0:dt:t], measuredStates(:,1:mStatesPlotIndex));
  legend('distanceTravelledEstimate', 'RadiusEstimate', 'pathCompensationEstimate');
  mStatesPlotIndex = mStatesPlotIndex +1;
  
  
end
subplot(3,2,1);
robotdraw(robotState(1),robotState(2),robotState(3), linearRobotStateEstimate(1), linearRobotStateEstimate(2), linearRobotStateEstimate(3)); %needed to hold last frame