%speedsteeringcontrol
%TODO show why system is unstable for speed < 2 * abs(steering)
function [Ul,Ur] = ssc(speed, steering, cUl, cUr) %inputs are desired speed and turn radius and current wheel speeds, returns new wheel speed to set.
 %has wierd bullshit if one speed approaches 0

Ur1 = speed;
Ul1 = speed;
dUr1 =0;
dUl1=0;

if (speed<0) %not default, makes theta linear with steering rather than y
    steering = -steering;
end;

if (steering>0 && cUl ~=0) % if steering right, and dont '/0'
  dUl1 = steering; %juice the left
  dUr1 = -cUr/cUl * dUl1; %prop the right
elseif (steering<0 && cUr ~=0)% if steering left, and dont '/0'
  dUr1 = -steering; %juice the right
  dUl1 = -cUl/cUr * dUr1; %prop the left
elseif (steering==0) %if only moving forward
  dUr1 = 0; 
  dUl1 =0;
elseif (speed ==0) %turning in place or not moving
  dUr1 = -steering;
  dUl1 = steering;
%elseif (cUl==0 || cUr ==0) %not moving
%  dUr1 = .1;
%  dUl1 = .1; %tappy tap tap
end;

if (speed<0) %default
    dUr1 = -dUr1;
    dUl1 = -dUl1;
end
%if (speed<0) %not default
%    tempp = dUr1;
%    dUr1 = -dUl1;
%    dUl1 = -tempp;
%end

Ur = Ur1 + dUr1;
Ul = Ul1 + dUl1;

%from Differential Speed Steering Control for Fout-Wheel independent Driving Electric Vehicle
% by Xiaodong Wu, Min Xu, and Lei Wang
%they came up with dL = -cUr/cUl *dR; where x and y are the right and left
%tires

%Additional logic, Austin F. Oltmanns
