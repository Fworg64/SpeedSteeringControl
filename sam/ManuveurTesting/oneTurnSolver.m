%oneturnsolver
%if a waypoint and robot pose pair match the conditions for a one turn
%manuever, this function retuturn the two maneuvers to get from the pose to
%the waypoint as two turns, one which is actually a straight segment

function [distance1, radius1, xc1, yc1, distance2, radius2, xc2, yc2] = oneTurnSolver(robotx, roboty, robotth, waypointx, waypointy, waypointth)

%convert waypoint to robotCoords
[wpX, wpY, wpTh] = transformPoseToRobotCoord(robotx, roboty, robotth, waypointx, waypointy, waypointth);

xintercept = -wpY / tan(wpTh) + wpX;

if (xintercept <0 || sign(wpTh) ~= sign(wpY))
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

distancetoendsq = (wpX - xintercept) * (wpX - xintercept) + wpY*wpY;
distancetostartsq = xintercept*xintercept;

if (distancetostartsq < distancetoendsq) %turn is first maneuver
    xc1 = 0;
    yc1 = tan((pi - wpTh)/2) * xintercept;
    radius1 = yc1;
    distance1 = abs(wpTh*radius1);
    
    xtangent = radius1 * cos(distance1/radius1 - pi/2); %point common to line and arc
    ytangent = radius1 * sin(distance1/radius1 - pi/2) + radius1;
    
    radius2 = 1000;
    distance2 = sqrt((xtangent - wpX)^2 + (ytangent - wpY)^2);
    
    xhalf = (wpX + xtangent)/2;
    yhalf = (wpY + ytangent)/2;
    %M = tan(wpTh);
    %xc2 = xhalf + sqrt(1000^2 / (M^2 +1));
    %yc2 = -1/M * (xc2 - xhalf) + yhalf;
    xc2 = xhalf + radius2 * cos(wpTh + pi/2);
    yc2 = yhalf + radius2 * sin(wpTh + pi/2);
    
    %DONT FORGET TO UNTRANSFORM!!
    [distance1,radius1,xc1,yc1] = transformManeuverToWorldCoord(robotx, roboty, robotth,distance1,radius1,xc1,yc1);
    [distance2,radius2,xc2,yc2] = transformManeuverToWorldCoord(robotx, roboty, robotth,distance2,radius2,xc2,yc2);
else %turn is second
    A = 1;
    B = (2*wpY / tan(wpTh) - 2*wpX);
    C = wpX^2 - 2*wpY*wpX/tan(wpTh) - wpY^2; %is this correct? %could this be turned into trig?
    
    potXcA = (-B + sqrt(B^2 - 4*A*C))/(2*A);
    potXcB = (-B - sqrt(B^2 - 4*A*C))/(2*A);
    xc2 = min(potXcA, potXcB);
    
    distance1 = xc2;
    radius1 = 1000;
    xc1 = xc2/2;
    yc1 = 1000;
    
    radius2 = -1/tan(wpTh) * (xc2 - wpX) +wpY;
    yc2 = radius2;
    distance2 = abs(wpTh*radius2);
    
    %DONT FORGET TO UNTRANSFORM!!
    [distance1,radius1,xc1,yc1] = transformManeuverToWorldCoord(robotx, roboty, robotth,distance1,radius1,xc1,yc1);
    [distance2,radius2,xc2,yc2] = transformManeuverToWorldCoord(robotx, roboty, robotth,distance2,radius2,xc2,yc2);

end
