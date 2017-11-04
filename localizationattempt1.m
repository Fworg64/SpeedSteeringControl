%localization filter attempt1

%strategy, get IMU data and VESC data. shove VESC data into kinemetic model
%of robot to get velocity estimates for dx, dy, dtheta
%integrate IMU data to get another set of dx, dy, dtheta
%estimate state with bayesian combination of this information
%implementation looks like
%residual = estimated states - measurements
%dEstimate = dynamics*estimatedstates + input +k*residual
%estimate = estimate + dEstimate

%then, get Tag position data for x, y, theta and integrate velocity estimate
%for another x, y, theta estimate. Bayesian filter them together to get 
%x, y and theta estimate.

dt = .01;
time = 0:dt:10;
axelLen = .5;
wheelDia = .33;

Ul = .3;
Ur = .2;

robotState = [0,0,0,0,0,0];
measuredRobotState = [0,0,0,0,0,0];
estimatedRobotState = [0,0,0,0,0,0];
dERS = [0,0,0,0,0,0];
residual = [0,0,0,0,0,0];
measuredWheelSpeeds = [0,0];

Kx = diag([.1, .1, .1, .1, .1, .1]);

trueRecord = zeros(6,length(time));
measuredRecord = zeros(6,length(time));
estimatedRecord = zeros(6,length(time));
recordIndex =0;

for t = time
    dx = robotdynamics(Ul,Ur,robotState(3),dt,wheelDia,axelLen);
    robotState = [robotState(1:3) + dx', dx'];
    measuredRobotState(1) = robotState(1) + normrnd(0,.02); %x, from tag
    measuredRobotState(2) = robotState(2) + normrnd(0,.02); %y, from tag
    measuredRobotState(3) = robotState(3) + normrnd(0,.02); %theta, from tag
    measuredRobotState(4) = robotState(4) + normrnd(0,.02); %xv, integrate IMU
    measuredRobotState(5) = robotState(5) + normrnd(0,.02); %yv, integrate IMU
    measuredRobotState(6) = robotState(6) + normrnd(0,.02); %theta, integrate IMU
    measuredWheelSpeeds = [Ul + normrnd(0, .02), Ur + normrnd(0,.02)]; %read back from vescs
    residual = estimatedRobotState - measuredRobotState;
    dERS1 = robotdynamics(measuredWheelSpeeds(1), measuredWheelSpeeds(2), estimatedRobotState(3), dt, wheelDia, axelLen);
    estimatedRobotState = [estimatedRobotState(1:3) + dERS1',dERS1'] - residual*Kx*dt;
    
    recordIndex = recordIndex+1;
    trueRecord(:,recordIndex ) = robotState;
    measuredRecord(:,recordIndex) = measuredRobotState;
    estimatedRecord(:,recordIndex) = estimatedRobotState;
end

figure()
tites = ['X';'Y';'Theta';'dX';'dY';'dTh']
for plotter = 1:6
  subplot(2,3,plotter);
  plot(time, trueRecord(plotter,:), time, measuredRecord(plotter,:), time, estimatedRecord(plotter,:));
  legend('True', 'measured', 'estimated');
  title(tites(plotter,:));
end
    