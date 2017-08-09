%sscv2

%Speed is .5*(LeftAngularVel + RightAngularVel)*WheelRadius; (angularVel in Rad)
%Turn Radius is DistanceBetweenWheels/2 * (LeftAngularVel + RightAngularVel)/(RightAngularVel - LeftAngularVel)

%Given Speed and Turn Radius, solve for Angular Velocities
function [Ul,Ur] = sscv2(speed, radius, wheelR, AxelLen) %inputs are desired speed and turn radius and current wheel speeds, returns new wheel speed to set.

if (radius ~=0)
  Ul = (4*radius*speed/(AxelLen*wheelR) - 2*speed/wheelR)*AxelLen/(radius*4);
  Ur = 2*speed/wheelR - Ul;
else
Ul = -.5*speed*wheelR;
Ur = .5*speed*wheelR;
end