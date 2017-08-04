% State-Space Simple Harmonic Oscillator
%
% This script simulates a damped simple harmonic oscillator (i.e. mass on a
% spring) using a state-space description, and also implements a linear
% observer.
%
% Tobin Fricke
% 2012-04-13

% Parameters

k = 1;  % spring constant [m/s]
m = 1;  % mass [kg]
b = 0;  % damping constant [kg/m/s]

% Define the state-space matrices

% The dynamic system is defined by these matrices:
%
% (d/dt) x = A x + B u
%        y = C x + D u
%
% where
%
% x: internal state
% u: input to system
% y: output from system

A = [0 1; (-k/m) (-b/m)];   % state --> state derivative
B = [0 ; (1/m)];            % input --> state derivative
C = [1 0];                  % state --> output 
D = 0;                      % input --> output

% Define the gains matrix for the observer

K = [0.5; -0.1];            % residual --> state hat dot

% Initial state

x = [1 ; 0];                % initial state
xhat = [0; 0];              % initial state estimate
t = 0;                      % initial time [s]
u = 0;                      % initial input 

% Simulation parameters

N = 251;                    % number of time steps
dt = 0.1;                   % time step [seconds]

% allocate space for the results

xs = nan(2, N);             % time series of true state
xhats = nan(2, N);          % time series of estimated state
ts = nan(1, N);             % time
 
% Do the simulation

for ii=1:length(xs)
  % record the state
  xs(:, ii) = x;
  xhats(:,ii) = xhat;
  ts(ii) = t;
  
  % get the current outputs
  y       = C*x + D*u;      % output of physical system
  yhat    = C*xhat + D*u;   % expected output
  r       = y - yhat;       % residual
  
  % advance the state
  x     = expm(A*dt)*(x + B*u*dt);
  xhat  = expm(A*dt)*(xhat + B*u*dt + K*r*dt);
  t     = t + dt;
  
end

% Plot the results

figure(1);
subplot(2,1,1);
plot(ts, xs(1,:), '.-', ts, xhats(1,:), 'o-');
legend('true', 'estimated');
ylabel('position [m]');
xlabel('time [s]');
grid on

subplot(2,1,2);
plot(ts, xs(2,:), '.-', ts, xhats(2,:),'o-');
legend('true', 'estimated');
ylabel('velocity [m/s]');
xlabel('time [s]');
grid on

figure(2);
plot(xhats(1,:), xhats(2,:), 'o-',  xs(1,:), xs(2,:), 'r-.');
title('phase space trajectory');
xlabel('position [m]');
ylabel('velocity [m/s]');
legend('estimated state', 'true state');
axis square
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
grid on