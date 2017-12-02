%Two turn solver
%the solver for any case that the oneTurnSolver does not solve

function [distance1, radius1, xc1, yc1, distance2, radius2, xc2, yc2] = twoTurnSolver(robotx, roboty, robotth, waypointx, waypointy, waypointth)

[wpX, wpY, wpTh] = transformPoseToRobotCoord(robotx, roboty, robotth, waypointx, waypointy, waypointth);

xintercept = -wpY / tan(wpTh) + wpX;

if (xintercept >=0 && sign(wpTh) ~= sign(wpY)) %opposite of oneTurnSolver if you called this with these conditions you did it wrong
    distance1 =0;
    radius1=0;
    xc1=0;
    yc1=0;
    distance2=0;
    radius2=0;
    xc2=0;
    yc2=0;
    return;
end

disp('TwoTurnSolver');

%polynomial from distance equation
%this is for solutions where goal is in positive Y plane
%that is, first radius is positive (to the left) and second radius is
%negative (to the right)
if (wpY > 0)
    cosanglearg = -wpTh - pi/2;
    sinanglearg = -wpTh +pi/2
    distance1sign = 1;
    distance2sign = 1;
else
    cosanglearg = -wpTh - pi/2;
    sinanglearg = -wpTh +pi/2;
    distance1sign = 1;
    distance2sign = 1;
end
A = cos(cosanglearg)^2 + (1+sin(sinanglearg))^2 -4;
B = -2*wpX*cos(cosanglearg) - 2*wpY*(1+sin(sinanglearg));
C = wpX^2 + wpY^2;

da = (-B + sqrt(B^2 - 4*A*C))/(2*A)
db = (-B - sqrt(B^2 - 4*A*C))/(2*A)

d = max(da,db);

xc1 =0;
yc1 = d;
radius1 = d
radius2 = -d;
xc2 = wpX - d*cos(cosanglearg)
yc2 = wpY - d*sin(sinanglearg)

xintermediate = (xc1+xc2)/2
yintermediate = (yc1+yc2)/2

theta1 = atan2(xintermediate, radius1-yintermediate)
distance1 = distance1sign*d*theta1   %atan2(yc2-yintermediate,xintermediate)
distance2 = distance2sign*d*(theta1-wpTh)      %(atan2(wpY - yc2,wpX - xc2) - atan2(wpY - yintermediate, wpX - xintermediate))

%display intermediate points in world coord
[xintWORLD, yintWORLD, thetaintWORLD] = transformPoseToRobotCoord(robotx, roboty, robotth, xintermediate, yintermediate, abs(distance1/radius1))

%     %Construct quatratic polynomial and determine roots
%     A = 2*(1-sin(pi/2 -wpTh));
%     B = 2*( wpX*cos(pi/2-wpTh) + wpY*(1+sin(pi/2-wpTh)) );
%     C = -(wpX^2 + wpY^2);
%     p = [A B C];
%     radii = roots(p); 
%     
%     r = max(radii); %shouldn't the radius be signed?
%     
%     radius1 = r;
%     radius2 = r; %one of these need to be negative
%     xc1 = 0;
%     yc1 = r;
%     
%     xc2 = wpX - r*cos((pi/2)-wpTh);
%     yc2 = wpY - r*sin((pi/2)-wpTh);
%     
%     distance1 = (pi/2) + atan((yc2-r)/xc2); 
%     distance2 = (pi/2) + atan((yc2-r)/xc2) + wpTh;

%DONT FORGET TO UNTRANSFORM!!
[distance1,radius1,xc1,yc1] = transformManeuverToWorldCoord(robotx, roboty, robotth,distance1,radius1,xc1,yc1);
[distance2,radius2,xc2,yc2] = transformManeuverToWorldCoord(robotx, roboty, robotth,distance2,radius2,xc2,yc2);

disp('End TwoTurnSolver');
end

