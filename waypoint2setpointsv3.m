%path solver, takes destination x,y,theta and returns number of different
%manuevers (1 or 2?) and their setpoints (distance and turn radius)

%if tan(theta)*(-x1)+y1 >0, then solution is one circular arc
% that is, if the waypoint line was extended back and intersects with the
% current position line after the waypoint.

%coordinate system: +X is fwd, +Y is Left

function [firstdistance, firstradius, seconddistance, secondradius] = waypoint2setpointsv3(x1, y1, theta)
%theta is in rad and ranges -pi to pi, x and y are in meters

%find x intercept
% y - y1 = m(x - x1)
% 0 -y1 = tan(theta)(x-x1)
% -y1 = tan(theta)*x - tan(theta)*x1
% -tan(theta)*x =y1 - tan(theta)*x1
% x = -y1/tan(theta) + x1

xintercept = -y1/tan(theta) + x1;

if (xintercept>=0 && (y1 >0 &&theta >0 && theta <pi || y1<0 && theta <0 && theta > -pi)) %if x intercept is greater than 0 and final orientation is facing away from starting center line
   %find intersection point and which point is closest
   %intersection point is (xintercept,0)
   %find distance to waypoint from intersection
    distancetoendsq = (x1 - xintercept)^2 + y1^2;
    distancetostartsq = xintercept^2
   
   %find bisecting line for intersection
   %angle is (PI+theta)/2
   %find point for center of circle on intersection of bisecting line and line perpindicular to closest orientation (waypoint or origin)
   
   if (distancetostartsq < distancetoendsq)
    %normal line is just the y axis
    %bisecting line is y - 0 = tan((pi+theta)/2)*(x-Xint)
    ycenter = tan((pi+theta)/2)*(-xintercept);
    xcenter=0;
    radius = ycenter;
   else
    %normal line is y - y1 = -1/tan(theta) * (x - x1)
    %bisecting line is y - 0 = tan((pi+theta)/2)*(x-Xint)
    xcenter = (-tan((pi+theta)/2)*xintercept - 1/tan(theta)*x1-y1)/(tan((pi+theta)/2) - 1/tan(theta));
    ycenter = tan((pi+theta)/2)*(xcenter-xintercept);
    radius = sqrt((x1-xcenter)^2 + (y1-ycenter)^2);
   end
   
   %next need to find the tangent point on the far side and then compute the arc length
   %after that, find the remaining distance to travel
   
   if (distancetostartsq <distancetoendsq)
     %tangent point will be on the waypoint line
     %waypoint eq : y  = tan(theta) * (x- x1) +y1
     %cirle eq: radius^2 = (x-xcenter)^2 + (y - ycenter)^2
     %%circle eq: x = sqrt(radius^2 - (y - ycenter)^2) + xcenter
     %%% combined eq: xtangent = sqrt(radius^2 - (tan(theta)*(xtangent -x1) + y1 -ycenter)^2) + xcenter;
     %%% xtangent = sqrt(radius^2 - (tan(theta)*xtangent + -tan(theta)*x1 + y1 + -ycenter)^2) + xcenter; %careful with sign and grouping knownterms
     %%% xtangent = sqrt(radius^2 - (tan(theta)*xtangent + KNOWNTERM)^2) + xcenter;
     %%% xtangent = sqrt(radius^2 - (tan(theta)*xtangent)^2 + KNOWNTERM*tan(theta)*xtangent + KNOWNTERM^2) + xcenter;
     %%% (xtangent - xcenter)^2 = radius^2 - (tan(theta)*xtangent)^2 + KNOWNTERM*tan(theta)*xtangent + KNOWNTERM^2;
     %%%(xtangent - xcenter)^2 + (tan(theta)*xtangent)^2 - KNOWNTERM*tan(theta)*xtangent = radius^2 + KNOWNTERM^2;
     %%% xtangent^2 - xcenter*xtangent + xcenter^2 + tan(theta)^2*xtangent^2 - KNOWNTERM * tan(theta)*xtangent = radius^2 + KNOWNTERM^2
     %%% (1 + tan(theta)^2) * (xtangent^2) + (-xcenter - KNOWNTERM * tan(theta))*xtangent = radius^2 + KNOWNTERM^2 - xcenter^2
     
     
     %%%%PROBLEMS HERE, check derivation...
     KNOWNTERM = -tan(theta)*x1 + y1 - ycenter;
     xtangent = (-(-xcenter -KNOWNTERM*tan(theta)) + sqrt((-xcenter-KNOWNTERM*tan(theta))^2 - 4*(1+tan(theta)^2)*(-radius^2-KNOWNTERM^2+xcenter^2)))/(2*(1-tan(theta)^2));
     xtangent = (-(-xcenter -KNOWNTERM*tan(theta)) - sqrt((-xcenter-KNOWNTERM*tan(theta))^2 - 4*(1+tan(theta)^2)*(-radius^2-KNOWNTERM^2+xcenter^2)))/(2*(1-tan(theta)^2));
     %should be equal
     ytangent = tan(theta)*xtangent - x1 + y1;
   else
     %tangent point will be on the start line (xaxis)
     %start line eq: y=0;
     %circle eq = radius^2 = (x-xcenter)^2 + (y-ycenter)^2
     %%xtangent = sqrt(radius^2 - (0-ycenter)^2) + xcenter;
     %%xtangent = -sqrt(radius^2 - (0-ycenter)^2) + xcenter;
     xtangent = xcenter;
     ytangent = 0;
   end
   
   %now compute arc length from start to tangent point or tangent point to end
   if (distancetostartsq <distancetoendsq)
     %arc is from start point to tangent point
     %find angle from start point drawn to center to tangent point drawn to center
     arcangle = pi/2 + atan((ytangent-radius)/xtangent);
     firstdistance = arcangle*radius;
     firstradius = radius;
     %also calculate second distnace and radius
     seconddistance = sqrt((xtangent - x1)^2 + (ytangent - y1)^2);
     secondradius = 100000; %infinity
   else
     %arc is from tangent point to waypoint
     %find angle from tangent point (drawn back to center to waypoint drawn back to center
     arcangle = pi/2 + atan((ytangent - ycenter)/(xtangent - xcenter));
     seconddistance = arcangle*radius;
     secondradius = radius;
     %also caluclate first distance and radius
     firstdistance = xtangent;
     firstradius = 100000; %infinity
   
   end

else
    firstdistance = 1;
    firstradius = 100000;
    seconddistance = 69;
    secondradius = 3;
    
end


