function [ wpRx, wpRy, wpRth ] = reflectWaypointAroundRobot( wpx, wpy, wpth, rx, ry, rth)
%REFLECTWAYPOINTAROUNDROBOT reflects a waypoint around the y axis of the
%robot
%   does it in world coordinates

%reflect wpx and wpy around y = tan(rth)*(x - xth) + yth
%wpRth = rth-anglediff(wpth,rth)
wpRth = rth-angleDiff(wpth,rth);

%from stackexchange
%put line in ay + bx + c =0
%y - y1 = m(x - x1)
%y = m*x -m*x1 +y1
%y - m*x + m*x1 -y1 =0
m = tan(rth);
a = 1;
b = -m;
c = m*rx - ry;

wpRx = (wpx*(a^2 - b^2) - 2*b*(a*wpy+c))/(a^2 + b^2);
wpRy = (wpy*(b^2 - a^2) - 2*a*(b*wpx+c))/(a^2 + b^2);


end
%%test! 
% [x, y, th] = reflectWaypointAroundRobot(0, -1, pi/3, 0, 0, 0)
% x =
%      0
% y =
%      1
% th =
%    -1.0472
% pi/3
% ans =
%     1.0472