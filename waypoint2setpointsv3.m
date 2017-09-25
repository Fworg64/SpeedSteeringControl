%path solver, takes destination x,y,theta and returns number of different
%manuevers (1 or 2?) and their setpoints (distance and turn radius)

%if tan(theta)*(-x1)+y1 >0, then solution is one circular arc
% that is, if the waypoint line was extended back and intersects with the
% current position line after the waypoint.

%coordinate system: +X is fwd, +Y is Left

function [firstdistance, firstradius, seconddistance, secondradius, xcenter, ycenter, xcenter2, ycenter2] = waypoint2setpointsv3(x1, y1, theta)
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
    distancetostartsq = xintercept^2;
   
   %find bisecting line for intersection
   %angle is (PI+theta)/2
   %find point for center of circle on intersection of bisecting line and line perpindicular to closest orientation (waypoint or origin)
   
   if (distancetostartsq < distancetoendsq) %intersection is closer to start, turn will be on first jaunt
    %normal line is just the y axis
    %bisecting line is y - 0 = tan((pi+theta)/2)*(x-Xint)
    ycenter = tan((pi+theta)/2)*(-xintercept);
    xcenter=0;
    radius = ycenter;
   else %intersection is closer to end, turn will be on second jaunt
    %circle center will be on intersection of line normal to waypoint and line normal to tangent point (common point between paths)
    %%xcenter2 = (-tan((pi+theta)/2)*xintercept - 1/tan(theta)*x1-y1)/(tan((pi+theta)/2) - 1/tan(theta)); %this is incorrect
    %%ycenter2 = tan((pi+theta)/2)*(xcenter2-xintercept); %wrong
    %eqn1: radius2^2 = (xcenter2 - x1)^2 + (radius2-y1)^2
    %eqn2: radius2 = -1/tan(theta) * (xcenter2 - x1) + y1;
    %eqnbonus: -(radius2 - y1)*tan(theta) +x1 = xcenter2; %transformed eqn2
    %bigeqn: radius2^2 == ((-(radius2 - y1)*tan(theta) +x1) - x1)^2 + (radius2-y1)^2
    %bigbadeqn: (-1/tan(theta) * (xcenter2 - x1) + y1)^2 == (xcenter2 + x1)^2 + ((-1/tan(theta) * (xcenter2 - x1) + y1)-y1)^2
    
    potxcenter2a = (-(2*y1/tan(theta) - 2*x1) + sqrt((2*y1/tan(theta) - 2*x1)^2 - 4*(x1^2 - 2*y1/tan(theta)*x1 - y1^2)))/2;
    potxcenter2b = (-(2*y1/tan(theta) - 2*x1) - sqrt((2*y1/tan(theta) - 2*x1)^2 - 4*(x1^2 - 2*y1/tan(theta)*x1 - y1^2)))/2;
    %pick smallest Xc >0;
    xcenter2 = -1;
    if (potxcenter2a < potxcenter2b)
      if (potxcenter2a >0)
        xcenter2 = potxcenter2a;
      end
    else
      if (potxcenter2b >0)
        xcenter2 = potxcenter2b;
      end
    end
    %if xcenter2 is -1, we have a problem
    radius = -1/tan(theta) * (xcenter2 - x1) + y1;
    ycenter2 = radius;
    %radius = sqrt((x1-xcenter2)^2 + (y1-ycenter2)^2); %correct but not good enough
   end
   
   %next need to find the tangent point on the far side and then compute the arc length
   %after that, find the remaining distance to travel
   
   if (distancetostartsq <distancetoendsq) %intersection is closer to start, straightaway will be on second jaunt
     %tangent point (common point between turn and straightaway) will be on the waypoint line and the intersection of line normal to waypoint line through cirlce center
     %waypoint eq : y  = tan(theta) * (x- x1) +y1
     %normal line through center of cirle
     % y = -1/tan(theta) * (x - xcenter) + ycenter;
     xtangent = ((tan(theta))^2 * x1 - y1*tan(theta) + xcenter + ycenter*tan(theta))/((tan(theta))^2+1);
     ytangent = tan(theta) * (xtangent - x1) + y1;
     
   else %intersection is closer to end, straightaway will be on first jaunt
     %tangent point (common point between turn and straightaway) will be on the start line (xaxis) and line perpindicular to it (y axis) through the circle center
     %start line eq: y=0;
     %circle eq = radius^2 = (x-xcenter)^2 + (y-ycenter)^2
     %%xtangent = sqrt(radius^2 - (0-ycenter)^2) + xcenter;
     %%xtangent = -sqrt(radius^2 - (0-ycenter)^2) + xcenter;
     xtangent = xcenter2;
     ytangent = 0;
   end
   
   %now compute arc length from start to tangent point or tangent point to end
   if (distancetostartsq <distancetoendsq) %intersection is closer to start, turn is on first jaunt
     %arc is from start point to tangent point
     %find angle from start point drawn to center to tangent point drawn to center
     arcangle = pi/2 + atan((ytangent-radius)/xtangent);
     firstdistance = arcangle*radius;
     firstradius = radius;
     %also calculate second distance and radius

     seconddistance = sqrt((xtangent - x1)^2 + (ytangent - y1)^2);
     secondradius = 100; %infinity
     
     %need to compute "center" of this turn as well
     %%should be half the distance forward and approximatly the radius out
     %%find point halfway between tangent point and destination
     %%its the average of coordinates, xhalf = (x1 - xtangent)/2; yhalf = (y1 - ytangent)/2;
     %%extend line perpindicular to this line radius units
     %% secondradius^2 = (xhalf - xcenter)^2 + (yhalf - ycenter)^2
     %% ycenter = -1/tan(theta) * (xcenter - xhalf) + yhalf
     
     %% secondradius^2 = (xhalf - xcenter)^2 + (yhalf - (-1/tan(theta) * (xcenter - xhalf) + yhalf))^2;
     
     xhalf = (x1 - xtangent)/2;
     yhalf = (y1 - ytangent)/2;
     
     xcenter2 = xhalf - secondradius^2 * (-1 + 1/(cos(theta)^2));
     xcenter2 = xhalf + secondradius^2 * (-1 + 1/(cos(theta)^2));
     
     %doesnt matter which xcenter2 is chosen because it is a straigt line actually

     ycenter2 = -1/tan(theta) * (xcenter - xhalf) + yhalf;
     
   else
     %arc is from tangent point to waypoint
     %find angle from tangent point to waypoint
     %arcangle = acos((xtangent - xcenter2)/radius) - acos((x1 - xcenter2)/radius); %note, in this case xtangent = xcenter2
     %care! acos has limited range, what quadrant is the circle?
     if ((x1 - xcenter2)/radius > 0) % on the right half of the circle
      if (ycenter2 > y1) %less than 1/4 circle, less than pi/2 rad
        arcangle = pi/2 - acos((x1 - xcenter2)/radius); %circle starts at bottom
      else % between 1/4 and 1/2 circle, pi/2 to pi rad
        arcangle = pi/2 + acos((x1 - xcenter2)/radius);
      end
     else %on the left half of the circle
       if (ycenter2 <y1) % between 1/2 circle and 3/4, pi to 3pi/2 rad
         arcangle = pi/2 +acos((x1 - xcenter2)/radius);
       else %almost the whole friggen circle
         arcangle = pi - acos((x1 - xcenter2)/radius) + 3*pi/2;
       end
     end
     seconddistance = arcangle*radius;
     secondradius = radius;
     %also caluclate first distance and radius
     %need to compute "center" of this turn as well
     %%same routine as above, but obviously which center and which distance and such are analogous, not the same
     %%also get special case that direction of travel is allong x axis, meaning half distance is xtangent/2 and y is just radius
     firstdistance = xtangent;
     firstradius = 100; %infinity
     
     xcenter = xtangent/2;
     ycenter = radius; %positive or negative, doesnt matter because it is actually a straight line
   
   end
   
   %%need to flip around center and center2

else
    %need two turn method
    firstdistance = 1;
    firstradius = 100000;
    seconddistance = 69;
    secondradius = 3;
    
end


