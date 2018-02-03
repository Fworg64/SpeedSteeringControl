%sscv3
%turns speed, turn radius, axel length and max speed
%into a pair of wheel velocities in M/s (not rad/s!)

%if speed is positive
%positive radius is forwards and towards the left
%negative radius is forwards and towards the right
%if speed is negative
%positve radius is backwards and to the right (same circle as positive
%speed)
%negative radius is backwards and to the left (same circle as positive
%speed)

%max speed is absolute value
%sign of speed indicates direction

%Speed is .5*(LeftAngularVel + RightAngularVel)*WheelRadius; (angularVel in Rad)
%Turn Radius is DistanceBetweenWheels/2 * (LeftAngularVel + RightAngularVel)/(RightAngularVel - LeftAngularVel)


function [Ul,Ur] = sscv3(speed, radius, AxelLen, MaxSpeed)

if (radius ~= 0)
  Ul = (4*radius*speed/(AxelLen) - 2*speed)*AxelLen/(radius*4);
  Ur = 2*(speed) - Ul;
  
  maxabs = max(abs(Ul), abs(Ur));
  if (maxabs > MaxSpeed)
      speed = speed * MaxSpeed/maxabs;
      Ul = (4*radius*speed/(AxelLen) - 2*speed)*AxelLen/(radius*4);
      Ur = 2*(speed) - Ul;
  end
else
  Ul = -.5*speed;
  Ur = .5*speed;
end

end