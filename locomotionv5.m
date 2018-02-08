%locomotionv5
figure();
%control robot

%reject disturbance of 2*pi/7 rad/s on wheels

Radius = -1.7;
center = [1,-1.7];
speed = .3;
MaxSpeed = .5;
wheelR = .3;
AxelLen = .5;
[LeftWheelSetSpeed, RightWheelSetSpeed] = sscv3(speed, Radius, AxelLen, .5)
LvelCmd = LeftWheelSetSpeed;
RvelCmd = RightWheelSetSpeed;
LwheelSpeed =0;
RwheelSpeed =0;
dt=.02;
plotPeriod = 20;
time = 0:dt:81;

robotPose = [1;.4;.1]; %x,y,theta

VescGains = [-4];
%control gains
EPlpGain = 0;%.00120;
EPlpAlpha = .00008;
%WheelAlphaFC = .0000001*2*pi /7;
%WheelAlpha =  2*pi*dt*WheelAlphaFC/(2*pi*dt*WheelAlphaFC+1); %put EP through LP filter and integrate that for long term stability
WheelAlpha = .5;
%wheel alpha = .5 at dt = .02 for speed = .3
%wheel alpha = .15 at dt = .02 for speed = .1
%EPiDecayGain = .0072*dt;

EPpGain = 0;%.01;%.0015;
EPdGain = .4; %lower number = better short term response, higher number = better long term stability
ETpGain = 0;%.0001;
ETdGain  =.45;%.120;
EPpLowPassGain = 2*pi*dt*.1608/(2*pi*dt*.1608+1); %.01; % 2*pi*dt*fc/ (2*pi*dt*fc+1)
ETpLowPassGain = 2*pi*dt*.1608/(2*pi*dt*.1608+1); %alpha for Fc @ dt
WheelSpeedPGain = 0;%.009;
%control states
EPpLowPass=0;
EPpLowPassPrev =0;
ETpLowPass=0;
ETpLowPassPrev=0;
EPpDerivFiltEst =0;
ETpDerivFiltEst=0;
EPLowPass =0;
EPLowPassPrev = 0;

wheelSpeedsPlot = zeros(2,length(time));
wheelCmdsPlot = zeros(2,length(time));
pathErrorPlot = zeros(3,length(time));
pathErrorFiltPlot = zeros(2,length(time));
ErrorDerivPlot = zeros(2,length(time));
plotIndex=0;
plotCounter =1;
%figure();

wheelDisturbance = ones(2,length(time));
disturbanceLength1S = int32(.5 / dt);
disturbanceLength2S = int32(5 / dt);



wheelDisturbance(1,int32(20/dt):((int32(20/dt) + disturbanceLength1S))) = .4;
wheelDisturbance(2,int32(40/dt):((int32(40/dt) + disturbanceLength2S))) = .3;

for t=time
     LwheelSpeed = (LwheelSpeed + VescGains(1)*(LwheelSpeed - LvelCmd)*dt)*wheelDisturbance(1,int16(t/dt)+1);
     RwheelSpeed = (RwheelSpeed + VescGains(1)*(RwheelSpeed - RvelCmd)*dt)*wheelDisturbance(2,int16(t/dt)+1);
     
     if (LwheelSpeed > MaxSpeed) LwheelSpeed = MaxSpeed; end
     if (RwheelSpeed > MaxSpeed) RwheelSpeed = MaxSpeed; end
     if (LwheelSpeed < -MaxSpeed) LwheelSpeed = -MaxSpeed; end
     if (RwheelSpeed < -MaxSpeed) RwheelSpeed = -MaxSpeed; end
     
     dRobot = robotdynamics(LwheelSpeed/wheelR,RwheelSpeed/wheelR, robotPose(3), dt, wheelR, AxelLen);
     robotPose = robotPose + dRobot;
     
     measuredRobotPose = robotPose + [normrnd(0,.02);normrnd(0,.02);normrnd(0,.02)];
     [CPPx,CPPy,CPPth] = newFindCPP(center(1), center(2), Radius, measuredRobotPose(1), measuredRobotPose(2));
     if (Radius >0)
       EPpEst = Radius - sqrt((center(1) - measuredRobotPose(1))^2 + (center(2) - measuredRobotPose(2))^2); %positive error means turn right, assume robot is pointing the correct direction
     else
       EPpEst = Radius + sqrt((center(1) - measuredRobotPose(1))^2 + (center(2) - measuredRobotPose(2))^2); %positive error means turn right, assume robot is pointing the correct direction
     end
     ETpEst = angleDiff(measuredRobotPose(3),CPPth);
     EPpLowPassPrev = EPpLowPass;
     EPpLowPass = EPpLowPassGain * EPpEst + (1- EPpLowPassGain) * EPpLowPassPrev;
     ETpLowPassPrev = ETpLowPass;
     ETpLowPass = (ETpLowPassGain * ETpEst + (1- ETpLowPassGain) * ETpLowPassPrev);
     
     EPLowPass = (EPlpAlpha * EPpEst + (1-EPlpAlpha) * EPLowPassPrev);
     EPLowPassPrev = EPLowPass;
     
     EPpDerivFiltEst = (EPpLowPass - EPpLowPassPrev)/dt;
     ETpDerivFiltEst = angleDiff(ETpLowPass,ETpLowPassPrev)/dt;

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
     
     %LaccelInput = ((EPpGain*EPpEst + EPlpGain*EPLowPass + EPdGain*EPpDerivFiltEst) - (ETpGain*ETpEst + ETdGain*ETpDerivFiltEst) - WheelSpeedPGain*(LvelCmd - LeftWheelSetSpeed))*dt;
     
     LvelCmdPrev = LvelCmd;
     RvelCmdPrev = RvelCmd;
     
     Lgas =  (sign(speed) * ((EPpGain*EPpEst + EPlpGain*EPLowPass + EPdGain*EPpDerivFiltEst) + sign(speed)*(ETpGain*ETpEst + ETdGain*ETpDerivFiltEst))) *dt;
     Rgas = -(sign(speed) * ((EPpGain*EPpEst + EPlpGain*EPLowPass + EPdGain*EPpDerivFiltEst) + sign(speed)*(ETpGain*ETpEst + ETdGain*ETpDerivFiltEst))) *dt;
     
     %LvelCmd = LvelCmd - ((EPpGain*EPpEst + EPlpGain*EPLowPass + EPdGain*EPpDerivFiltEst) - (ETpGain*ETpEst + ETdGain*ETpDerivFiltEst) - WheelSpeedPGain*(LvelCmd - LeftWheelSetSpeed))*dt;
     %RvelCmd = RvelCmd + ((EPpGain*EPpEst + EPlpGain*EPLowPass + EPdGain*EPpDerivFiltEst) - (ETpGain*ETpEst + ETdGain*ETpDerivFiltEst) - WheelSpeedPGain*(RvelCmd - RightWheelSetSpeed))*dt;
     
     LvelCmd = LvelCmd + Lgas;
     RvelCmd = RvelCmd + Rgas;
     
     LvelCmd = WheelAlpha * LvelCmd + (1-WheelAlpha) * LvelCmdPrev;
     RvelCmd = WheelAlpha * RvelCmd + (1-WheelAlpha) * RvelCmdPrev;
     
     %scale LvelCmd and RvelCmd if CPPx^2 + CPPy^2 is something...
     %do this by passing through sscv3?
     %%instantaneosTurnRadius = AxelLen/2 * (LvelCmd + RvelCmd)/(RvelCmd - LvelCmd);
     %%errorSpeedScale = 1; %change this based on some distance error metric
     %errorSpeedScale = 1/exp(4*EPpEst^2); %this shapes some of the damping of the response
     %maybe increase radius with distance error
     %%errorRadiusScale = 1; %correct radius with sign of EPpEst
     %errorRadiusScale = -.01*EPpEst +1;
     %errorRadiusScale = -.1 * EPpEst^3 +1;
     %[LvelCmd, RvelCmd] = sscv3(speed * errorSpeedScale, instantaneosTurnRadius * errorRadiusScale, AxelLen, MaxSpeed);
     
     plotIndex = plotIndex+1;
     wheelSpeedsPlot(:,plotIndex) = [LwheelSpeed;RwheelSpeed];
     wheelCmdsPlot(:,plotIndex) = [LvelCmd, RvelCmd];
     pathErrorPlot(:,plotIndex) = [EPpEst;ETpEst;EPLowPass];
     pathErrorFiltPlot(:,plotIndex) = [EPpLowPass;ETpLowPass];
     ErrorDerivPlot(:,plotIndex) = [EPpDerivFiltEst;ETpDerivFiltEst];
     %plot robot
     if (t>plotCounter*plotPeriod)
         plotCounter = plotCounter+1; 
         subplot(2,2,1);
         hold on;
         if (Radius >0)
           rectangle('Position',[center(1)-Radius center(2)-Radius 2*Radius 2*Radius],'Curvature',[1,1]);
         else
           rectangle('Position',[center(1)+Radius center(2)+Radius -2*Radius -2*Radius], 'Curvature', [1,1]);
         end
         robotdraw(robotPose(1),robotPose(2),robotPose(3),0,0,0);
         %hold off;
         title('Robot Tracing 3m Diameter Circle from (1,0)');xlabel('X Distance (m)');ylabel('Y Distance (m)');
         subplot(2,2,2);
         plot([0:dt:t], wheelSpeedsPlot(:,1:plotIndex), '-', [0:dt:t], wheelCmdsPlot(:,1:plotIndex), '--')
         legend('Left Wheel Speed', 'Right Wheel Speed', 'Left Wheel Cmd', 'Right Wheel Cmd');
         title('Wheel Speeds'); xlabel('Time (s)');ylabel('Velocity (m/s)');
         subplot(2,2,3);
         plot([0:dt:t], pathErrorPlot(:,1:plotIndex),'-', [0:dt:t], pathErrorFiltPlot(:,1:plotIndex),'*');
         axis([0,t,-.5,.5])
         legend('Mea. Path Error', 'Mea. Angle Error', 'Mea. Path Error LP', 'Filt Path Error', 'Filt Angle Error');
         title('Measured States w/o Noise');xlabel('Time (s)');ylabel(sprintf('Path Error (m\nAngle Error (rad)'));
         subplot(2,2,4);
         plot([0:dt:t], ErrorDerivPlot(:,1:plotIndex),'--')
         legend('Path Filt Deriv', 'Angle Filt Deriv');
         axis([0,t,-.2,.2])
         title('Derivative of Errors');xlabel('Time (s)');ylabel(sprintf('Path Error Derivative (m/s)\nAngle Error Derivative (rad/s)'));
     end
    %get all sensor data
    %do SS for each error state
    %set wheel accel is sum of 3 error states
    %integrate accel for wheel vel cmd
end