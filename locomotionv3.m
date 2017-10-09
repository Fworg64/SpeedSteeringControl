%%locomotionv3
%this script demonstrates the control system that will be used on the NDSU
%NRMC Robot for 2017/2018

%derivitive of path error?
%turn radius compensation term
%calculate turn radius, take it in as ameasured state
%compare it to the set turn radius
%modify speed based on difference... maybe through it into path
%compensation term?
%or should it be its own state?
%angle of robot with respect to path as a seperate state <---=|
%something with theta and the center of the circle and the xp,yp
%paththeta = atan2(yp - center(2),xp - center(1))
%robottheta = theta
%need to add derivitive of this state too for stability

InitialRobotState = [1;0;0]; %x,y,theta
robotPhysicalPose = InitialRobotState; %x,y,theta used in graph
robotStateEstimate = [0;0;0;0;0;0;0;0]; 
measuredRobotState = robotStateEstimate; %for now;
trueRobotState = [0;0;0;0;0;0;0];
prevMeasuredRobotState5 =0;
prevRobotStateEstimate4 =0;
derivOfEst4 =0;

U=0;

waypoint = [4;1;pi/4] %world coordinates
%transform waypoint to robot coordinates
waypointTrans = waypoint - InitialRobotState;
waypointRot = [cos(InitialRobotState(3)), sin(InitialRobotState(3));-sin(InitialRobotState(3)),cos(InitialRobotState(3))] *  [waypointTrans(1);waypointTrans(2)];
waypointRobotCoord = [waypointRot(1);waypointRot(2);waypointTrans(3)]

[Distance, Radius, Distance2, Radius2, xcenterUT, ycenterUT, xcenter2UT, ycenter2UT] = waypoint2setpointsv3(waypointRobotCoord(1),waypointRobotCoord(2),waypointRobotCoord(3))

robotStateEstimate(6) = Distance;
measuredRobotState(6) = Distance;
prevMeasuredRobotState5 = Distance;
trueRobotState(6) = Distance;

%transform turn centers to world coordinates
%translate, then rotate
center1 = [xcenterUT;ycenterUT] + [InitialRobotState(1);InitialRobotState(2)];
center1 =  [cos(InitialRobotState(3)), sin(InitialRobotState(3));-sin(InitialRobotState(3)),cos(InitialRobotState(3))] * [center1(1);center1(2)]

center2 = [xcenter2UT; ycenter2UT]+ [InitialRobotState(1);InitialRobotState(2)];
center2 =  [cos(InitialRobotState(3)), sin(InitialRobotState(3));-sin(InitialRobotState(3)),cos(InitialRobotState(3))] * [center2(1);center2(2)]

firstwaypointmet =0;
burstindex=0;
pertubationDistance = .05; %leash length

dt = .01;
time = 0:dt:10;

wheelR = .3;
AxelLen = .5;

SpeedInput = 0;
SteeringInput = 0;
SteeringAccelInput=0;

%plotting records
crumbcounter = 21;
crumbperiod =20;
crumbindex=0;
crumbs = zeros(2,int32(length(time)/crumbperiod));

Uplot = zeros(3,length(time));
Uplotindex=1;
measuredStatesPlot = zeros(6,length(time));
mStatesPlotIndex =1;
posPlot = zeros(6, length(time));
posPlotindex=1;
%derivOfEst4Plot = zeros(1,length(time));
%d4PlotIndex=1;

%states are
linearRobotA = [-2,0,-.002,-.05,-120,0,0,-1; %left wheel speed
                0,-2,.002,.05,120,0,0,1; %right wheel speed
                0,0,-100,8,0,0,0,0; %steering correction
                .1,-.1,0,-.8,0,0,0,0; %path error
                0,0,0,0,-.3,0,0,0; %derivitive of path error
                -.005,-.005,0,0,0,-.2,0,0; % distance from closest point on path to goal
                -.005,-.005,0,0,0,0,-.2,0; %distance to closest point on path
                0,0,0,0,0,0,0,-.2];
            %leftwheel speed has friction, accepts steering input, and gets
            %regular input
            %rightwheel speed has friction, accepts steering input, and
            %gets regular input
            %steering correction integrates path error and decays and
            %decreases with input
            %path error is increasd by the wheels spinning a certain way
            %and decays
            %derivitive of path error...
            %not sure if it can be described right now
            %it is taken from the estimated states and input as a
            %measurement
            %distance from closest point on path to goal
            %should decrease with wheel speed and decays
            %distance to closest point on path
            %should decrease with wheel speed and decays
            
            %closest point on path
            %%should be at least turnradius/10 in front of the robot
            %%should never "go back"
            %%must always travel in the initial direction towards the goal
            %%must ultimatly shoot past the goal
            
            
linearRobotB = [1;1;0;0;0;0;0;0];

 linearRobotQ = [40,0,0,0,0,0,0,0; %weighting for lqr gains
                0,40,0,0,0,0,0,0;
                0,0,1,0,0,0,0,0;
                0,0,0,10,0,0,0,0;
                0,0,0,0,10,0,0,0;
                0,0,0,0,0,10000,0,0;
                0,0,0,0,0,0,1,0
                0,0,0,0,0,0,0,1];
linearRobotR = 1;
PlantRoots = eig(linearRobotA)
%Kx = lqr(linearRobotA, linearRobotB, linearRobotQ, linearRobotR)
%ServoRoots = eig(linearRobotA - linearRobotB*Kx) %currently unused
%LQRRoots = eig(linearRobotA - linearRobotB * Kx)
%pick observer gains
%to make it a kalman-bucy filter, use lqr methods to make them proportional to the noise of each sensor
ObserverGains = [1;1;6;8;6;1;1;6];
Residual = [0;0;0;0;0;0;0;0]; %the unweighted difference between the estimated states and the measured states

for t = time;
    dRobotState = robotdynamics(trueRobotState(1),trueRobotState(2), robotPhysicalPose(3), dt, wheelR, AxelLen);
    robotPhysicalPose = robotPhysicalPose + dRobotState;
    
    trueRobotState(1) = robotStateEstimate(1); %for now, true robotState might be a lie
    trueRobotState(2) = robotStateEstimate(2); %for now
    
    %wheel manager (clip if they go to far)
    %if (trueRobotState(1) <0)
    %    trueRobotState(1) = 0;
    %end
    %if (trueRobotState(2) <0)
    %    trueRobotState(2) =0;
    %end
    %if (trueRobotState(1) > 10)
    %    trueRobotState(1) =10;
    %end
    %if (trueRobotState(2) >10)
    %    trueRobotState(2) =10;
    %end
    
    measuredRobotState(1) = trueRobotState(1); %+noise, this comes from the VESC
    measuredRobotState(2) = trueRobotState(2); %+noise, this comes from the VESC
    %measuredRobotState(3) is not measured, because it is just an integral
    %term
     %get path compensation term first by finding nearest point on path and taking signed difference between robot pose and closest path point
  %%nearest point on path is point on path that is on the line through the center of the circle and the robots pose that is closest to the robots pose.
  %%%calculating path compensation term
  %%care! need to be careful about which solution is grabbed, dont want the other side of the circle
  %%this could all be done in world coordinates, which means waypointsolver center needs to be transformed through robots initial pose to the world
  %thisLineEq: y - center1(2) = (center1(2) - robotState(2))/(center1(1) - robotState(1)) * (x - center1(1));
  %pathEq: Radius^2 = (x - center1(1))^2 + (y - center1(2))^2;
  %will be the solution that is closest to robot pose - this is the care!
  if (firstwaypointmet ==0)
    if (center1(1) - robotPhysicalPose(1) ==0)
      M = (center1(2) - robotPhysicalPose(2))/.001;
    else
      M = (center1(2) - robotPhysicalPose(2))/(center1(1) - robotPhysicalPose(1));
    end
    potXpa = (-(-2*center1(1) - 2*M^2*center1(1)) + sqrt((-2*center1(1) - 2*M^2*center1(1))^2 - 4*(1+M^2)*(center1(1)^2 - Radius^2 + M^2*center1(1)^2)))/(2*(1 + M^2));
    potXpb = (-(-2*center1(1) - 2*M^2*center1(1)) - sqrt((-2*center1(1) - 2*M^2*center1(1))^2 - 4*(1+M^2)*(center1(1)^2 - Radius^2 + M^2*center1(1)^2)))/(2*(1 + M^2));
    if (abs(robotPhysicalPose(1) - potXpa) < abs(robotPhysicalPose(1) - potXpb))
     Xp = potXpa;
    else
     Xp = potXpb;
    end
    Yp = M*(Xp - center1(1)) + center1(2);

  else
    if (center2(1) - robotPhysicalPose(1) ==0)
     M = (center2(2) - robotPhysicalPose(2))/.001;
    else
     M = (center2(2) - robotPhysicalPose(2))/(center2(1) - robotPhysicalPose(1));
    end
    potXpa = (-(-2*center2(1) - 2*M^2*center2(1)) + sqrt((-2*center2(1) - 2*M^2*center2(1))^2 - 4*(1+M^2)*(center2(1)^2 - Radius2^2 + M^2*center2(1)^2)))/(2*(1 + M^2));
    potXpb = (-(-2*center2(1) - 2*M^2*center2(1)) - sqrt((-2*center2(1) - 2*M^2*center2(1))^2 - 4*(1+M^2)*(center2(1)^2 - Radius2^2 + M^2*center2(1)^2)))/(2*(1 + M^2));
    if (abs(robotPhysicalPose(1) - potXpa) < abs(robotPhysicalPose(1) - potXpb))
     Xp = potXpa;
    else
     Xp = potXpb;
    end
    Yp = M*(Xp - center2(1)) + center2(2);

  end
  %reminder that this is being measured in steering...
  %this number will be negative if a larger turn radius is needed
  %and positive if a smaller turn radius is needed. So subtract it. 
  %not actually true, need to make sure this always points towards a point
  %on the path so far away
  if (firstwaypointmet ==0)
    pathCompensationEstimate = sqrt((center1(1) - robotPhysicalPose(1))^2 + (center1(2) - robotPhysicalPose(2))^2) - Radius;
  else
    pathCompensationEstimate = sqrt((center2(1) - robotPhysicalPose(1))^2 + (center2(2) - robotPhysicalPose(2))^2) - Radius2;
  end
  %strategy: find arc angle from xp and yp, never let it decrease,
  %and keep the furthest robotPhysicalPose that got you that
  %then, measure if the furthest path point is  would have a bigger radius or a
  %smaller radius then the current robot position

    %now grab point at this angle, or you should have kept the point when
    %you had it
  
  %dont%thresh pathCompensation
  %pathCompensationEstimate = 10*pathCompensationEstimate;
  %if (pathCompensationEstimate > .5)
  %    pathCompensationEstimate = .5;
  %elseif (pathCompensationEstimate < -.5)
  %    pathCompensationEstimate = -.5;
  %end
    
  measuredRobotState(4) = pathCompensationEstimate;
  
  derivOfEst4 = robotStateEstimate(4) - prevRobotStateEstimate4;
  prevRobotStateEstimate4 = robotStateEstimate(4);
  
  measuredRobotState(3) = 0;%measuredRobotState(3) + measuredRobotState(4)*dt; %force this to 0
  
  measuredRobotState(5) = 1000*derivOfEst4*dt;
  %get distance traveled from closest path point and back calculating distance travelled along arc
  %acos gives 0 to pi for 1 to -1
  %asin gives pi/2 to -pi/2 for 1 to -1
  %distance is arcangle * r, r is given
  %arcangle is angle of closest point on circle - angle of startpoint on circle
  %angle of startpoint on circle is -pi/2 for first jaunt and distance1/Radius1 - pi/2 for second jaunt
  if (firstwaypointmet ==0)
    %%need to be careful about which quadrant life is in, can test with xp and yp relative to xc and yc
    %%only really need to know top half or bottom half
    arcangle = (acos((Xp - center1(1))/Radius) - pi/2);
    %want to perturb angle such that pertubationangle*radius = Xcm
    %so pertubationangle = Xcm/radius;
    %%%pertubationangle = pertubationDistance/Radius;
    %%%arcangle = arcangle + pertubationangle;
    if (Yp > center1(2))
      arcangle = arcangle + pi;
    end
    distanceTravelledEstimate = -Radius * arcangle;
    measuredRobotState(6) = Distance - distanceTravelledEstimate + pertubationDistance;
  else
    arcangle = (asin((Xp - center2(1))/Radius2) - (Distance/Radius - pi/2));
    %%pertubationangle = pertubationDistance/Radius2;
    %%arcangle = arcangle + pertubationangle;
    if (Yp > center2(2))
      arcangle = arcangle + pi;
    end
    distanceTravelledEstimate = -Radius2 * arcangle;
    measuredRobotState(6) = Distance2 - distanceTravelledEstimate + pertubationDistance;
  end
  
  %never let distance to goal increase
  if (measuredRobotState(6) > prevMeasuredRobotState5)
      measuredRobotState(6) = prevMeasuredRobotState5;
  end
  
  
  measuredRobotState(7) = sqrt((robotPhysicalPose(1) - Xp)^2 + (robotPhysicalPose(2) - Yp)^2);
  if (firstwaypointmet ==0)
    measuredRobotState(8) = atan2(Yp - center1(2),Xp - center1(1)) - robotPhysicalPose(3) + pi/2;
  else
    measuredRobotState(8) = atan2(Yp - center2(2),Xp - center2(1)) - robotPhysicalPose(3) +pi/2;
  end
  
  %get Residual after measureing states
  Residual = measuredRobotState - robotStateEstimate;
  
  dlinearRobot = linearRobotA * robotStateEstimate + linearRobotB * U + ObserverGains.* Residual;% + setpoint if any
  robotStateEstimate = robotStateEstimate + dlinearRobot*dt;
  
  %change this to the e^distance_towards_goal  - 1;
  %if (abs(robotStateEstimate(1)) + abs(robotStateEstimate(2)) <10) 
  %    U = -Kx*robotStateEstimate;% <- after servo is added to distance?
  %end
  %U = 
  %if (exp(robotStateEstimate(6)) - 1 >5)
  %    U = 5;
  %else
  %    U = exp(robotStateEstimate(6)) - 1;
  %end
  %U = 3*(robotStateEstimate(6) - 10*robotStateEstimate(5))
  
  burstwidth = .1;
  burstspace =.5;
  
  if (sin(2*pi*t/burstwidth) > .2)
      U = 5;
  else
      U=0;
  end
  
     %switch setpoints if distance has been travelled
   if ( t > .2 && measuredRobotState(5) < .05 && firstwaypointmet ==0) 
    firstwaypointmet =1;
    prevMeasuredRobotState5 = Distance2;
   end

    %plot robot
  subplot(2,2,1);
  hold on;
  robotdraw(robotPhysicalPose(1),robotPhysicalPose(2),robotPhysicalPose(3),0,0,0);
  %%plot path points
  %scatter(path(1,:),path(2,:),'green','*');
  %record crumbs
  crumbcounter = crumbcounter+1;
  if (crumbcounter>crumbperiod)
    crumbcounter=0;
    crumbindex = crumbindex+1;
    crumbs(:,crumbindex) = robotPhysicalPose(1:2);
  end
  scatter(crumbs(1,1:crumbindex),crumbs(2,1:crumbindex));
  if (firstwaypointmet ==1)
      text(5,2,'First Waypoint Met');
      %plot(center2(1),center2(2),'b*');
      plot([center2(1),robotPhysicalPose(1),Xp], [center2(2), robotPhysicalPose(2),Yp],'b--',[center2(1),robotPhysicalPose(1),Xp], [center2(2), robotPhysicalPose(2),Yp],'b*');
  else
      %plot(center1(1),center1(2),'g*');
      plot([center1(1),robotPhysicalPose(1),Xp], [center1(2), robotPhysicalPose(2),Yp],'g--',[center1(1),robotPhysicalPose(1),Xp], [center1(2), robotPhysicalPose(2),Yp],'g*');
  end
  coords = sprintf('x: %d\ny: %d\n\theta: %d', robotPhysicalPose(1), robotPhysicalPose(2), robotPhysicalPose(3));
  text(5,1,coords);
  coords = sprintf('xp: %d\nyp: %d', Xp, Yp);
  text(5,0,coords);
  hold off;
    %plot motor inputs
  subplot(2,2,2);
  Uplot(:,Uplotindex) = [U;trueRobotState(1);trueRobotState(2)];
  plot([0:dt:t],Uplot(:,1:Uplotindex));
  legend('Gas', 'LeftWheel Speed', 'Right Wheel Speed');
  Uplotindex = Uplotindex+1;
  
  %plot estimated states, minus inputs
    subplot(2,2,3);
  posPlot(:,posPlotindex) = [robotStateEstimate(3:8)'];
  plot([0:dt:t], posPlot(1:3, 1:posPlotindex), '-', [0:dt:t], posPlot(4:6, 1:posPlotindex), '--');
  legend('Steering Correction CW', 'Path Error Term', 'Estimated Deriv of Perror','Distance to goal', 'Distance to closest point','Estimated Angle Diff');
  posPlotindex = posPlotindex+1; 
  title('Estimated States');
  
  %plot measured states
  subplot(2,2,4);
  measuredStatesPlot(:,mStatesPlotIndex) = measuredRobotState(3:8);
  %derivOfEst4Plot(d4PlotIndex) = derivOfEst4;
  plot([0:dt:t], measuredStatesPlot(:,1:mStatesPlotIndex));
  legend('Steering Correction CW', 'Path Error Term', '10xDerivOfEst4', 'Distance to goal', 'Distance to closest point', 'Measured Angle Diff');
  mStatesPlotIndex = mStatesPlotIndex +1;
  %d4PlotIndex = d4PlotIndex +1;
  title('Measured States');
  
end
subplot(2,2,1);
robotdraw(robotPhysicalPose(1),robotPhysicalPose(2),robotPhysicalPose(3),0,0,0);
