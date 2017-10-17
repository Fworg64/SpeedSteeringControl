%locomotionv5

%control robot

Radius = 1.5;
center = [1,1.5];
speed = .05;
wheelR = .3;
AxelLen = .5;
[LeftWheelSetSpeed, RightWheelSetSpeed] = sscv2(speed, Radius, wheelR, AxelLen, 20)
LvelCmd = LeftWheelSetSpeed;
RvelCmd = RightWheelSetSpeed;
LwheelSpeed =0;
RwheelSpeed =0;
dt=.01;
time = 0:dt:40;

robotPose = [1;0;0]; %x,y,theta

VescGains = [-8];

EPpGain = .02;
EPdGain = .16;
ETpGain = .0005;
ETdGain  =.0040;
WheelSpeedPGain = .018;

EPpLowPassGain = .01;
ETpLowPassGain = .01;

EPpLowPass=0;
EPpLowPassPrev =0;
ETpLowPass=0;
ETpLowPassPrev=0;
EPpDerivFiltEst =0;
ETpDerivFiltEst=0;

wheelSpeedsPlot = zeros(2,length(time));
wheelCmdsPlot = zeros(2,length(time));
pathErrorPlot = zeros(2,length(time));
pathErrorFiltPlot = zeros(2,length(time));
ErrorDerivPlot = zeros(2,length(time));
plotIndex=0;
plotPeriod = .5;
plotCounter =1;
figure();

wheelDisturbance = ones(2,length(time));
wheelDisturbance(1,1000:1300) = .1;
wheelDisturbance(2,2500:3000) = .1;

for t=time
     LwheelSpeed = (LwheelSpeed + VescGains(1)*(LwheelSpeed - LvelCmd)*dt)*wheelDisturbance(1,int16(t/dt)+1);
     RwheelSpeed = (RwheelSpeed + VescGains(1)*(RwheelSpeed - RvelCmd)*dt)*wheelDisturbance(2,int16(t/dt)+1);
     dRobot = robotdynamics(LwheelSpeed,RwheelSpeed, robotPose(3), dt, wheelR, AxelLen);
     robotPose = robotPose + dRobot;
     
     measuredRobotPose = robotPose + .1*rand(3,1);
     [CPPx,CPPy,CPPth] = findCPP(measuredRobotPose(1), measuredRobotPose(2), center(1), center(2), Radius);
     EPpEst = Radius - sqrt((center(1) - measuredRobotPose(1))^2 + (center(2) - measuredRobotPose(2))^2); %positive error means turn right, assume robot is pointing the correct direction
     ETpEst = calculateDifferenceBetweenAngles(measuredRobotPose(3),CPPth); %be careful with difference in angles here pi=0;
     
     EPpLowPassPrev = EPpLowPass;
     EPpLowPass = EPpLowPassGain * EPpEst + (1- EPpLowPassGain) * EPpLowPassPrev;
     ETpLowPassPrev = ETpLowPass;
     ETpLowPass = ETpLowPassGain * ETpEst + (1- ETpLowPassGain) * ETpLowPassPrev;
     
     EPpDerivFiltEst = (EPpLowPass - EPpLowPassPrev)/dt;
     ETpDerivFiltEst = (ETpLowPass - ETpLowPassPrev)/dt;

     if EPpEst > .2
         EPpEst = .2;
     elseif EPpEst <-.2;
         EPpEst = -.2;
     end
     
     if ETpEst > pi/2
         ETpEst = pi/2;
     elseif ETpEst < -pi/2
         ETpEst = -pi/2;
     end
     
     LvelCmd = LvelCmd + (EPpGain*EPpEst + EPdGain*EPpDerivFiltEst) - (ETpGain*ETpEst + ETdGain*ETpDerivFiltEst) - WheelSpeedPGain*(LvelCmd - LeftWheelSetSpeed);
     RvelCmd = RvelCmd - (EPpGain*EPpEst + EPdGain*EPpDerivFiltEst) + (ETpGain*ETpEst + ETdGain*ETpDerivFiltEst) - WheelSpeedPGain*(RvelCmd - RightWheelSetSpeed);
     
     plotIndex = plotIndex+1;
     wheelSpeedsPlot(:,plotIndex) = [LwheelSpeed;RwheelSpeed];
     wheelCmdsPlot(:,plotIndex) = [LvelCmd, RvelCmd];
     pathErrorPlot(:,plotIndex) = [EPpEst;ETpEst];
     pathErrorFiltPlot(:,plotIndex) = [EPpLowPass;ETpLowPass];
     ErrorDerivPlot(:,plotIndex) = [EPpDerivFiltEst;ETpDerivFiltEst];
     %plot robot
     if (t>plotCounter*plotPeriod)
         plotCounter = plotCounter+1; 
         subplot(2,2,1);
         %hold on;
         robotdraw(robotPose(1),robotPose(2),robotPose(3),0,0,0);
         %hold off;
         subplot(2,2,2);
         plot([0:dt:t], wheelSpeedsPlot(:,1:plotIndex), '-', [0:dt:t], wheelCmdsPlot(:,1:plotIndex), '--')
         legend('Left Wheel Speed', 'Right Wheel Speed', 'Left Wheel Cmd', 'Right Wheel Cmd');
         subplot(2,2,3);
         plot([0:dt:t], pathErrorPlot(:,1:plotIndex),'-', [0:dt:t], pathErrorFiltPlot(:,1:plotIndex),'*',[0:dt:t], ErrorDerivPlot(:,1:plotIndex),'--');
         legend('Mea. Path Error', 'Mea. Angle Error', 'Filt Path Error', 'Filt Angle Error', 'Path Filt Deriv', 'Angle Filt Deriv');
     end
    %get all sensor data
    %do SS for each error state
    %set wheel accel is sum of 3 error states
    %integrate accel for wheel vel cmd
end
   