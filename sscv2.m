%%sscv2 Speed Steering Control V2
%Gives wheel velocities for a differential drive device given a desired speed and turn radius.
%Takes vehicle parameters of wheel radius and axel length as well as max allowable speed for a wheel to spin
%returned speeds are angular velocities in Rad/s

%Speed is .5*(LeftAngularVel + RightAngularVel)*WheelRadius; (angularVel in Rad)
%Turn Radius is DistanceBetweenWheels/2 * (LeftAngularVel + RightAngularVel)/(RightAngularVel - LeftAngularVel)

%Given Speed and Turn Radius, solve for Angular Velocities
function [Ul,Ur] = sscv2(speed, radius, wheelR, AxelLen, MaxSpeed) %inputs are desired speed and turn radius and current wheel speeds, returns new wheel speed to set.

if (radius ~=0) %(abs(radius) >=AxelLen/2)
  Ul = (4*radius*speed/(AxelLen*wheelR) - 2*speed/wheelR)*AxelLen/(radius*4);
  %Ul = (2*speed*(2*radius/(AxelLen*wheelR) - 1/wheelR)*AxelLen/radius*4);
  Sfactor =1;
  if (abs(Ul) >MaxSpeed) %if left wheel would be spinning too fast
    Sfactor = Ul/MaxSpeed;
    Ul = Ul / Sfactor; % scale wheel speed to 15 (the max)
  end
  Ur = 2*(speed/Sfactor)/wheelR - Ul; % scale other wheel proportionally
  
  if (abs(Ur) >MaxSpeed) %if right wheel would be spinning too fast
    S2factor = Ur/MaxSpeed;
    Ur = Ur/S2factor;
    %speed2 = speed/S2factor;
    %Ul = (4*radius*speed2/(AxelLen*wheelR) - 2*speed2/wheelR)*AxelLen/(radius*4);
    Ul = Ul/S2factor;
  end
else
Ul = -.5*speed*wheelR;
Ur = .5*speed*wheelR;
end

if (speed < 0)
Ul = -abs(Ul);
Ur = -abs(Ur);
end

%as radius approaches 0, wheel RPM goes to infinity to satisfy speed setting...
%need to limit speed as turn radius decreases